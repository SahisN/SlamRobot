#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node {
    public:
        SimpleSubscriber() : Node("simple_subscriber") {
            sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSubscriber::msgCallback, this, _1));
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

        void msgCallback(const std_msgs::msg::String &msg) const {
            RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data.c_str());
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // creates simple subscriber object
    auto node = std::make_shared<SimpleSubscriber>();

    // loops program
    rclcpp::spin(node);

    // destorys pointer on shutdown
    rclcpp::shutdown();

    return 0;
}