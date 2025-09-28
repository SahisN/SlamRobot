#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_motion {
    class PDMotionPlanner : public rclcpp::Node {
        public:
            PDMotionPlanner();
        
        private:
            // used for recieving path from planner node (subscribes to planner node)
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

            // used for sending robot velocity commands to ardunio
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

            // used for sending visual updates of robot moving from point a to b
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            rclcpp::TimerBase::SharedPtr control_loop_;

            /** PD Algorithm variables */ 
            // determines speed to move based on how far the target is from the robot
            double kp_;

            // determines how much to adjust speed based on how fast the robot is moving toward or away from the target
            double kd_;

            // determines how much time passes between each control update
            double step_size_;

            // max speed of robot when moving at straight line (per second)
            double max_linear_velocity_;

            // max speed of robot rotating around a point or axis (per second)
            double max_angular_velocity_;

            nav_msgs::msg::Path global_plan_;

            void controlLoop();
            void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
            bool transformPlan(const std::string &frame);
    };
}