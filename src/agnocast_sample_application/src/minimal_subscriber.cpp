#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "agnocast/agnocast_ioctl.hpp"  // MAX_SUBSCRIBER_NUM

using std::placeholders::_1;

static int64_t subscriber_count = 0;

class MinimalSubscriber : public rclcpp::Node
{
  agnocast::Subscription<agnocast_sample_interfaces::msg::Int64>::SharedPtr sub_;

  void callback(
    const agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::Int64> & message)
  {
    RCLCPP_INFO(this->get_logger(), "subscribe message: id=%ld", message->id);
  }

public:
  explicit MinimalSubscriber()
  : Node("minimal_subscriber_" + std::to_string(subscriber_count++))
  {
    rclcpp::CallbackGroup::SharedPtr group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions agnocast_options;
    agnocast_options.callback_group = group;

    sub_ = agnocast::create_subscription<agnocast_sample_interfaces::msg::Int64>(
      this, "/my_topic", 1, std::bind(&MinimalSubscriber::callback, this, _1), agnocast_options);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  agnocast::SingleThreadedAgnocastExecutor executor;
  for (int64_t i = 0; i < MAX_SUBSCRIBER_NUM; ++i) {
    executor.add_node(std::make_shared<MinimalSubscriber>());
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
