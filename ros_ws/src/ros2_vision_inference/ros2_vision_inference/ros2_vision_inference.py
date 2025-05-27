from rknnlite.api import RKNNLite
import numpy as np
import rclpy
import cv2
from .utils.ros_util import ROSInterface
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, CompressedImage
import threading
import time
from typing import List, Tuple

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class Metric3DThread(threading.Thread):
    def __init__(self, rknn_model_path):
        super().__init__()
        self.rknn_model_path = rknn_model_path
        self.build_model()
    
    def build_model(self):
        self.rknn_lite = RKNNLite()
        print('--> Load RKNN model')
        ret = self.rknn_lite.load_rknn(self.rknn_model_path)
        if ret != 0:
            print('Load RKNN model failed')
            exit(ret)
        print('RKNN model loaded successfully')
        
        # Init runtime environment
        print('--> Init runtime environment')
        ret = self.rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        if ret != 0:
            print('Init runtime environment failed')
            exit(ret)
        print('Runtime environment initialized')
    
    def set_inputs(self, image, P=None, timestamp=None):
        self.image = image
        self.P = P
        self.timestamp = timestamp

    def run(self):
        start_time = time.time()
        # Prepare input for model inference
        rknn_input, pad_info = self.prepare_input(self.image)
        
        # Perform inference using RKNN
        print('--> Running RKNN model')
        outputs = self.rknn_lite.inference(inputs=[rknn_input])
        print('RKNN inference done')
        
        depth_image = outputs[0][0, 0] # [1, 1, H, W] -> [H, W]
        
        depth_image = depth_image[pad_info[0] : depth_image.shape[0] - pad_info[1], pad_info[2] : depth_image.shape[1] - pad_info[3]] # [H, W] -> [h, w]
        self._output = depth_image, self.timestamp
        print(f"metric3d runtime: {time.time() - start_time}")

    def prepare_input(self, rgb_image: np.ndarray) -> Tuple[np.ndarray, List[int]]:
        input_size = (544, 1216)

        h, w = rgb_image.shape[:2]
        scale = min(input_size[0] / h, input_size[1] / w)
        self.scale = scale
        rgb = cv2.resize(rgb_image, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_LINEAR)

        padding = [123.675, 116.28, 103.53]
        h, w = rgb.shape[:2]
        pad_h = input_size[0] - h
        pad_w = input_size[1] - w
        pad_h_half = pad_h // 2
        pad_w_half = pad_w // 2
        rgb = cv2.copyMakeBorder(rgb, pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half, cv2.BORDER_CONSTANT, value=padding)
        pad_info = [pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half]

        # Convert to RGB format for RKNN
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        rgb = np.expand_dims(rgb, 0)  # Add batch dimension
        
        return rgb.astype(np.uint8), pad_info
    
    def join(self):
        threading.Thread.join(self)
        threading.Thread.__init__(self)
        return self._output
    
    def __del__(self):
        if hasattr(self, 'rknn_lite'):
            self.rknn_lite.release()


class VisionInferenceNode():
    def __init__(self):
        self.ros_interface = ROSInterface("VisionInferenceNode")

        self.logger = self.ros_interface.get_logger()
        self.clock = self.ros_interface.get_clock()
        self._read_params()
        self._init_model()
        self._init_static_memory()
        self._init_topics()

        self.logger.info("Initialization Done")
        self.ros_interface.spin()

    def _read_params(self):
        self.logger.info("Reading parameters...")
        
        self.metric3d_weight_path = self.ros_interface.read_one_parameters("METRIC3D_RKNN_FILE",
                                    "/home/orangepi/ros_ws/src/ros2_vision_inference/weights/metric3d.rknn")

    def _init_model(self):
        self.logger.info("Initializing RKNN model...")
        self.metric3d_thread = Metric3DThread(self.metric3d_weight_path)
        self.logger.info("RKNN Model initialized")
    
    def _init_static_memory(self):
        self.logger.info("Initializing static memory...")
        self.frame_id = None
        self.P = None
    
    def _init_topics(self):
        self.ros_interface.create_publisher(Image, "/cam0/depth_image", 10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.ros_interface.create_subscription(CameraInfo, "/cam0/camera_info", self.camera_info_callback, qos_profile=qos_profile)
        self.ros_interface.create_subscription(Image, "/cam0/image_raw", self.camera_callback, 1)

    def camera_info_callback(self, msg: CameraInfo):
        self.P = np.zeros((3, 4))
        self.P[0:3, 0:3] = np.array(msg.k.reshape((3, 3)))
        self.frame_id = msg.header.frame_id

    def process_image(self, image: np.ndarray, timestamp):
        starting = time.time()
        
        self.metric3d_thread.set_inputs(image, self.P.copy(), timestamp)
        self.metric3d_thread.start()
        
        depth_result = self.metric3d_thread.join()
        
        self.logger.info(f"Total runtime: {time.time() - starting}")

        # publish depth and point cloud
        if depth_result is not None:
            depth_image, timestamp = depth_result
            self.ros_interface.publish_image(depth_image, 
                                           image_topic="/cam0/depth_image", 
                                           frame_id=self.frame_id,
                                           timestamp=timestamp)

    def camera_callback(self, msg: Image):
        if self.P is None:
            self.logger.info("Waiting for camera info...", throttle_duration_sec=0.5)
            return # wait for camera info
        
        height = msg.height
        width  = msg.width
        
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))[:, :, ::-1]
        
        self.process_image(image, msg.header.stamp)


def main(args=None):
    rclpy.init(args=args)
    VisionInferenceNode()


if __name__ == "__main__":
    main()