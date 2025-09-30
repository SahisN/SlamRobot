#include <chrono>
#include "tf2/utils.h"
#include "bumperbot_motion/pure_pursuit.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bumperbot_motion {
    PurePursuit::PurePursuit() : Node("pure_pursuit_motion_planner_node"), 
    look_ahead_distance_(0.5), max_linear_velocity_(0.3), max_angular_velocity_(1.0) {
        declare_parameter<double>("look_ahead_distance", look_ahead_distance_);
        declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
        declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

        look_ahead_distance_ = get_parameter("look_ahead_distance").as_double();
        max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_= get_parameter("max_angular_velocity").as_double();

        path_sub_ = create_subscription<nav_msgs::msg::Path>("/a_star/path", 10, std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/target_pose", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        control_loop_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PurePursuit::controlLoop, this));
    }

    // updates global_plan_ when a new path is recieved (point a to point b from a* star publisher)
    void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {
        global_plan_ = *path;
    }

    void PurePursuit::controlLoop() {
        if(global_plan_.poses.empty()) {
            return;
        }

        geometry_msgs::msg::TransformStamped robot_pose;
        try {
            robot_pose = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        }

        catch (tf2::TransformException &error) {
            RCLCPP_WARN(get_logger(), "Could not transform : %s", error.what());
            return;
        }

        if(!transformPlan(robot_pose.header.frame_id)) {
            RCLCPP_ERROR(get_logger(), "Unable to transform Plan in robot's frame");
            return;
        }

        geometry_msgs::msg::PoseStamped robot_pose_stamped;
        robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
        robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
        robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
        robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;
        auto target_pose = getTargetPose(robot_pose_stamped);

        double dx = target_pose.pose.position.x - robot_pose_stamped.pose.position.x;
        double dy = target_pose.pose.position.y - robot_pose_stamped.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // if the distance difference between robot and the goal is less than 10cm
        // robot has reached the distance
        if(distance <= 0.1) {
            RCLCPP_INFO(get_logger(), "Goal Reached!");
            global_plan_.poses.clear();
            return;
        }

        target_pose_pub_->publish(target_pose);

        tf2::Transform robot_tf, target_pose_tf, target_pose_robot_tf;
        tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
        tf2::fromMsg(target_pose.pose, target_pose_tf);
        target_pose_robot_tf = robot_tf.inverse() * target_pose_tf;
        tf2::toMsg(target_pose_robot_tf, target_pose.pose);
        double curvature = getCurvature(target_pose.pose);

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = max_linear_velocity_;
        cmd_vel.angular.z = curvature * max_angular_velocity_;
        cmd_pub_->publish(cmd_vel);
    }

    bool PurePursuit::transformPlan(const std::string &frame) {
        if(global_plan_.header.frame_id == frame) {
            return true;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
        }

        catch(tf2::LookupException &error) {
            RCLCPP_ERROR_STREAM(get_logger(), "Couldn't transform plan from frame " << global_plan_.header.frame_id << " to " << frame);
            return false;
        }

        for(auto &pose : global_plan_.poses) {
            tf2::doTransform(pose, pose, transform);
        }

        global_plan_.header.frame_id = frame;
        return true;
    }

    geometry_msgs::msg::PoseStamped PurePursuit::getTargetPose(const geometry_msgs::msg::PoseStamped &robot_pose) {
        geometry_msgs::msg::PoseStamped target_pose = global_plan_.poses.back();
        
        // iterates in reverse order, starting from last to first pose
        for(auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it) {
            // calculate distance between pose_it and robot_pose
            double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
            double dy = pose_it->pose.position.y - robot_pose.pose.position.y;
            
            // using eudcian distance
            double distance = std::sqrt(dx * dx + dy * dy);

            // if the distances is within look_ahead_distance threshold, return the position
            if(distance > look_ahead_distance_) {
                target_pose = *pose_it;
            }

            else {
                break;
            }
        }

        return target_pose;
    }

    double PurePursuit::getCurvature(const geometry_msgs::msg::Pose &target_pose) {
        const double L = (target_pose.position.x * target_pose.position.x) + (target_pose.position.y * target_pose.position.y);

        if(L > 0.001) {
            return 2.0 * target_pose.position.y / L;
        }

        return 0.0;
    }

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}