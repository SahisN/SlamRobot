#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_motion {
    class PurePursuit : public rclcpp::Node {
        public:
            PurePursuit();
        
        private:
            // used for recieving path from planner node (subscribes to planner node)
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

            // used for sending robot velocity commands to ardunio
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

            // used for sending visual updates of robot moving from point a to b
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            rclcpp::TimerBase::SharedPtr control_loop_;

            /** Pure Pursuit Alogirthm variables */
            
            // determines how far robot can see to place a temporary goal position
            double look_ahead_distance_;

            // max speed of robot when moving at straight line (per second)
            double max_linear_velocity_;

            // max speed of robot rotating around a point or axis (per second)
            double max_angular_velocity_;

            nav_msgs::msg::Path global_plan_;

            // saves previous linear overshoot
            double prev_linear_error_;

            // saves previous angular overshoot
            double prev_angular_error_;

            // saves the previous cycle time for control loop function
            rclcpp::Time last_cycle_time_;

            void controlLoop();
            void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
            bool transformPlan(const std::string &frame);
            geometry_msgs::msg::PoseStamped getTargetPose(const geometry_msgs::msg::PoseStamped &robot_pose);
            double getCurvature(const geometry_msgs::msg::Pose &target_pose);
    };
}