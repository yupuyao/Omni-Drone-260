#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/quadrotor_msgs/msg/position_command.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>      
#include <sys/stat.h>    
#include <sys/mman.h>
#include <unistd.h> 

class TrajectoryInputNode : public rclcpp::Node
{
public:
    TrajectoryInputNode() : Node("trajectory_input_node")
    {

        trajectory_setpoint_publisher_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>(
            "/drone_0_planning/pos_cmd", 4);

        RCLCPP_INFO(this->get_logger(), "Trajectory Input Node started");
        RCLCPP_INFO(this->get_logger(), "Ready to accept coordinate inputs (x, y, z)");
        
        input_thread_ = std::thread(&TrajectoryInputNode::inputLoop, this);
    }

    ~TrajectoryInputNode()
    {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void inputLoop()
    {
        double x, y, z;
        
        while (rclcpp::ok()) {
            std::cout << "\n input cordinate (x y z)，or input 'q' quit: ";
            
            std::string input_line;
            std::getline(std::cin, input_line);
            
            if (input_line == "q" || input_line == "Q") {
                RCLCPP_INFO(this->get_logger(), "land...");
                const char* shm_name = "/shared_mem";

            int shm_fd = shm_open(shm_name, O_RDWR, S_IRUSR | S_IWUSR);
            if (shm_fd == -1) {
            std::cerr << "Failed to open shared memory" << std::endl;
            }

            void* shared_mem = mmap(nullptr, sizeof(std::atomic<bool>), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
           if (shared_mem == MAP_FAILED) {
           std::cerr << "Failed to map shared memory" << std::endl;
            }

            auto* land_triggered_ = reinterpret_cast<std::atomic<bool>*>(shared_mem);
                land_triggered_->store(true);
                munmap(shared_mem, sizeof(std::atomic<bool>));
                rclcpp::shutdown();
                break;
            }
            
            std::istringstream iss(input_line);
            if (iss >> x >> y >> z) {
                publishTrajectorySetpoint(x, y, z);
            } else {
                std::cout << "input form error, please input (x y z)" << std::endl;
            }
        }
    }

    void publishTrajectorySetpoint(double x, double y, double z)
    {
        auto msg = quadrotor_msgs::msg::PositionCommand();
        
        msg.position.x = static_cast<float>(x);
        msg.position.y = static_cast<float>(y);
        msg.position.z = static_cast<float>(z);
        
        trajectory_setpoint_publisher_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), 
                   "publish trajectory point: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    }

    rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr trajectory_setpoint_publisher_;
    std::thread input_thread_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TrajectoryInputNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
