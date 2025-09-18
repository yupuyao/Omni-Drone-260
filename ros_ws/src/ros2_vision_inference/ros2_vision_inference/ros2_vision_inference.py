from rknnlite.api import RKNNLite
import numpy as np
import rclpy
import cv2
from .utils.ros_util import ROSInterface
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, CompressedImage
import threading
import time
import queue
import concurrent.futures
from typing import List, Tuple, Optional
from dataclasses import dataclass
from collections import defaultdict

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


@dataclass
class InferenceTask:

    image: np.ndarray
    P: Optional[np.ndarray]
    timestamp: any
    task_id: int


@dataclass
class InferenceResult:

    depth_image: np.ndarray
    timestamp: any
    task_id: int
    processing_time: float


class Metric3DWorker:

    def __init__(self, worker_id: int, rknn_model_path: str, core_mask: int):
        self.worker_id = worker_id
        self.rknn_model_path = rknn_model_path
        self.core_mask = core_mask
        self.rknn_lite = None
        self.is_initialized = False
        self._init_model()
    
    def _init_model(self):
        
        try:
            self.rknn_lite = RKNNLite()
            print(f'Worker {self.worker_id}: Loading RKNN model')
            ret = self.rknn_lite.load_rknn(self.rknn_model_path)
            if ret != 0:
                print(f'Worker {self.worker_id}: Load RKNN model failed')
                return
            
            # 初始化运行时环境
            print(f'Worker {self.worker_id}: Init runtime environment')
            ret = self.rknn_lite.init_runtime(core_mask=self.core_mask)
            if ret != 0:
                print(f'Worker {self.worker_id}: Init runtime environment failed')
                return
            
            self.is_initialized = True
            print(f'Worker {self.worker_id}: RKNN model initialized successfully')
        except Exception as e:
            print(f'Worker {self.worker_id}: Initialization failed: {e}')
    
    def process_task(self, task: InferenceTask) -> Optional[InferenceResult]:
   
        if not self.is_initialized:
            return None
        
        start_time = time.time()
        
        try:
            # 准备输入
            rknn_input, pad_info = self._prepare_input(task.image)
            
            # 执行推理
            outputs = self.rknn_lite.inference(inputs=[rknn_input])
            
            # 处理输出
            depth_image = outputs[0][0, 0]  # [1, 1, H, W] -> [H, W]
            depth_image = depth_image[
                pad_info[0] : depth_image.shape[0] - pad_info[1], 
                pad_info[2] : depth_image.shape[1] - pad_info[3]
            ]
            
            processing_time = time.time() - start_time
            
            return InferenceResult(
                depth_image=depth_image,
                timestamp=task.timestamp,
                task_id=task.task_id,
                processing_time=processing_time
            )
            
        except Exception as e:
            print(f'Worker {self.worker_id}: Processing failed for task {task.task_id}: {e}')
            return None
    
    def _prepare_input(self, rgb_image: np.ndarray) -> Tuple[np.ndarray, List[int]]:
    
        input_size = (544, 1216)

        h, w = rgb_image.shape[:2]
        scale = min(input_size[0] / h, input_size[1] / w)
        rgb = cv2.resize(rgb_image, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_LINEAR)

        padding = [123.675, 116.28, 103.53]
        h, w = rgb.shape[:2]
        pad_h = input_size[0] - h
        pad_w = input_size[1] - w
        pad_h_half = pad_h // 2
        pad_w_half = pad_w // 2
        rgb = cv2.copyMakeBorder(rgb, pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half, cv2.BORDER_CONSTANT, value=padding)
        pad_info = [pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half]

        # 转换为RGB格式
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        rgb = np.expand_dims(rgb, 0)  # 添加批次维度
        
        return rgb.astype(np.uint8), pad_info
    
    def __del__(self):
        if hasattr(self, 'rknn_lite') and self.rknn_lite is not None:
            self.rknn_lite.release()


class AsyncMetric3DManager:
 
    def __init__(self, rknn_model_path: str, num_workers: int = 3, preserve_order: bool = True):
        self.rknn_model_path = rknn_model_path
        self.num_workers = num_workers
        self.preserve_order = preserve_order
        self.task_counter = 0
        self.task_counter_lock = threading.Lock()
        
        # 创建线程池和工作线程
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=num_workers)
        self.workers = []
        self._init_workers()
        
        # 结果队列和时序管理
        self.result_queue = queue.Queue()
        self.pending_tasks = {}
        self.pending_tasks_lock = threading.Lock()
        
        # 时序保证相关
        if self.preserve_order:
            self.completed_results = {}  # {task_id: result}
            self.next_output_id = 0      # 下一个应该输出的task_id
            self.results_lock = threading.Lock()
            self.max_buffer_size = 50    # 最大缓存结果数量
        
        # 性能统计
        self.stats = {
            'total_tasks': 0,
            'completed_tasks': 0,
            'failed_tasks': 0,
            'avg_processing_time': 0.0,
            'out_of_order_count': 0,
            'buffer_overflow_count': 0
        }
        self.stats_lock = threading.Lock()
    
    def _init_workers(self):
 
        # 为不同的工作线程分配不同的NPU核心
        core_masks = [
            RKNNLite.NPU_CORE_0,
            RKNNLite.NPU_CORE_1, 
            RKNNLite.NPU_CORE_2,
            RKNNLite.NPU_CORE_0_1_2
        ]
        
        for i in range(self.num_workers):
            core_mask = core_masks[i % len(core_masks)]
            worker = Metric3DWorker(i, self.rknn_model_path, core_mask)
            self.workers.append(worker)
    
    def submit_task(self, image: np.ndarray, P: Optional[np.ndarray], timestamp) -> int:
  
        with self.task_counter_lock:
            task_id = self.task_counter
            self.task_counter += 1
        
        task = InferenceTask(
            image=image.copy(),
            P=P.copy() if P is not None else None,
            timestamp=timestamp,
            task_id=task_id
        )
        
        # 选择负载最小的工作线程
        worker = self._get_best_worker()
        
        # 提交任务到线程池
        future = self.executor.submit(worker.process_task, task)
        future.add_done_callback(lambda f: self._handle_result(f, task_id))
        
        with self.pending_tasks_lock:
            self.pending_tasks[task_id] = {
                'future': future,
                'submit_time': time.time()
            }
        
        with self.stats_lock:
            self.stats['total_tasks'] += 1
        
        return task_id
    
    def _get_best_worker(self) -> Metric3DWorker:

        return self.workers[self.task_counter % len(self.workers)]
    
    def _handle_result(self, future: concurrent.futures.Future, task_id: int):
 
        try:
            result = future.result()
            
            if result is not None:
                if self.preserve_order:
                    self._handle_ordered_result(result)
                else:
                    self.result_queue.put(result)
                
                with self.stats_lock:
                    self.stats['completed_tasks'] += 1
                    # 更新平均处理时间
                    total = self.stats['completed_tasks']
                    self.stats['avg_processing_time'] = (
                        (self.stats['avg_processing_time'] * (total - 1) + result.processing_time) / total
                    )
            else:
                with self.stats_lock:
                    self.stats['failed_tasks'] += 1
                    
        except Exception as e:
            print(f"Task {task_id} failed: {e}")
            with self.stats_lock:
                self.stats['failed_tasks'] += 1
        
        # 清理待处理任务
        with self.pending_tasks_lock:
            self.pending_tasks.pop(task_id, None)
    
    def _handle_ordered_result(self, result: InferenceResult):

        with self.results_lock:
            # 存储完成的结果
            self.completed_results[result.task_id] = result
            
            # 检查缓存大小，防止内存泄漏
            if len(self.completed_results) > self.max_buffer_size:
                # 丢弃过旧的结果
                oldest_id = min(self.completed_results.keys())
                if oldest_id < self.next_output_id:
                    self.completed_results.pop(oldest_id, None)
                    with self.stats_lock:
                        self.stats['buffer_overflow_count'] += 1
            
            # 按顺序输出所有可以输出的结果
            while self.next_output_id in self.completed_results:
                ordered_result = self.completed_results.pop(self.next_output_id)
                self.result_queue.put(ordered_result)
                self.next_output_id += 1
    
    def get_result(self, timeout: float = 0.1) -> Optional[InferenceResult]:

        try:
            return self.result_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_stats(self) -> dict:

        with self.stats_lock:
            return self.stats.copy()
    
    def get_queue_size(self) -> int:
   
        with self.pending_tasks_lock:
            return len(self.pending_tasks)
    
    def get_buffer_status(self) -> dict:
   
        if not self.preserve_order:
            return {'buffer_size': 0, 'next_output_id': 0}
        
        with self.results_lock:
            return {
                'buffer_size': len(self.completed_results),
                'next_output_id': self.next_output_id,
                'max_buffer_size': self.max_buffer_size
            }
    
    def shutdown(self):

        self.executor.shutdown(wait=True)


class VisionInferenceNode:
    def __init__(self):
        self.ros_interface = ROSInterface("VisionInferenceNode")
        self.logger = self.ros_interface.get_logger()
        self.clock = self.ros_interface.get_clock()
        
        self._read_params()
        self._init_model()
        self._init_static_memory()
        self._init_topics()
        self._start_result_processor()

        self.logger.info("Initialization Done")
        self.ros_interface.spin()

    def _read_params(self):
        self.logger.info("Reading parameters...")
        
        self.metric3d_weight_path = self.ros_interface.read_one_parameters(
            "METRIC3D_RKNN_FILE",
            "/home/orangepi/ros_ws/src/ros2_vision_inference/weights/metric3d.rknn"
        )
        
        # 新增参数：工作线程数量和时序保证
        self.num_workers = self.ros_interface.read_one_parameters("NUM_WORKERS", 3)
        self.max_queue_size = self.ros_interface.read_one_parameters("MAX_QUEUE_SIZE", 10)
        self.preserve_order = self.ros_interface.read_one_parameters("PRESERVE_ORDER", True)

    def _init_model(self):
        self.logger.info("Initializing async RKNN manager...")
        self.inference_manager = AsyncMetric3DManager(
            self.metric3d_weight_path, 
            num_workers=self.num_workers,
            preserve_order=self.preserve_order
        )
        order_mode = "enabled" if self.preserve_order else "disabled"
        self.logger.info(f"Async RKNN manager initialized with {self.num_workers} workers, order preservation {order_mode}")
    
    def _init_static_memory(self):
        self.logger.info("Initializing static memory...")
        self.frame_id = None
        self.P = None
        self.last_stats_time = time.time()
    
    def _init_topics(self):
        self.ros_interface.create_publisher(Image, "/cam0/depth_image", 10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.ros_interface.create_subscription(
            CameraInfo, "/cam0/camera_info", 
            self.camera_info_callback, qos_profile=qos_profile
        )
        self.ros_interface.create_subscription(
            Image, "/cam0/image_rgb", 
            self.camera_callback, 1
        )

    def _start_result_processor(self):
    
        self.result_processor_thread = threading.Thread(
            target=self._process_results, daemon=True
        )
        self.result_processor_thread.start()

    def _process_results(self):
  
        while True:
            result = self.inference_manager.get_result()
            if result is not None:
                self._publish_result(result)
            
            # 定期输出统计信息
            current_time = time.time()
            if current_time - self.last_stats_time > 5.0:  # 每5秒输出一次
                self._log_stats()
                self.last_stats_time = current_time
            
            time.sleep(0.001)  # 短暂休眠避免CPU占用过高

    def _publish_result(self, result: InferenceResult):
  
        try:
            self.ros_interface.publish_image(
                result.depth_image,
                image_topic="/cam0/depth_image",
                frame_id=self.frame_id,
                timestamp=result.timestamp
            )
            
            self.logger.debug(
                f"Published result for task {result.task_id}, "
                f"processing time: {result.processing_time:.3f}s"
            )
        except Exception as e:
            self.logger.error(f"Failed to publish result: {e}")

    def _log_stats(self):
    
        stats = self.inference_manager.get_stats()
        queue_size = self.inference_manager.get_queue_size()
        buffer_status = self.inference_manager.get_buffer_status()
        
        log_msg = (
            f"Stats - Total: {stats['total_tasks']}, "
            f"Completed: {stats['completed_tasks']}, "
            f"Failed: {stats['failed_tasks']}, "
            f"Queue: {queue_size}, "
            f"Avg time: {stats['avg_processing_time']:.3f}s"
        )
        
        if self.preserve_order:
            log_msg += (
                f", Buffer: {buffer_status['buffer_size']}/{buffer_status['max_buffer_size']}, "
                f"Next ID: {buffer_status['next_output_id']}"
            )
            if stats['out_of_order_count'] > 0:
                log_msg += f", OOO: {stats['out_of_order_count']}"
            if stats['buffer_overflow_count'] > 0:
                log_msg += f", Overflow: {stats['buffer_overflow_count']}"
        
        self.logger.info(log_msg)

    def camera_info_callback(self, msg: CameraInfo):
        self.P = np.zeros((3, 4))
        self.P[0:3, 0:3] = np.array(msg.k).reshape((3, 3))
        self.frame_id = msg.header.frame_id

    def camera_callback(self, msg: Image):
        if self.P is None:
            self.logger.info("Waiting for camera info...", throttle_duration_sec=0.5)
            return

        # 检查队列大小，避免积压过多任务
        queue_size = self.inference_manager.get_queue_size()
        if queue_size > self.max_queue_size:
            self.logger.warn(
                f"Dropping frame due to queue overflow ({queue_size} > {self.max_queue_size})",
                throttle_duration_sec=1.0
            )
            return
        
        height = msg.height
        width = msg.width
        
        # 转换图像格式
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))[:, :, ::-1]
        
        # 异步提交推理任务
        task_id = self.inference_manager.submit_task(image, self.P, msg.header.stamp)
        
        self.logger.debug(f"Submitted task {task_id}, queue size: {queue_size}")

    def __del__(self):

        if hasattr(self, 'inference_manager'):
            self.inference_manager.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:
        VisionInferenceNode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
