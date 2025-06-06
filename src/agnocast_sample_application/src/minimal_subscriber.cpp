#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "agnocast/agnocast_ioctl.hpp"  // MAX_SUBSCRIBER_NUM

using std::placeholders::_1;

static int64_t g_subscriber_count = 0;

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
  : Node("minimal_subscriber_" + std::to_string(g_subscriber_count++))
  {
    const int64_t topic_id = g_subscriber_count / MAX_SUBSCRIBER_NUM;

    rclcpp::CallbackGroup::SharedPtr group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions agnocast_options;
    agnocast_options.callback_group = group;

    sub_ = agnocast::create_subscription<agnocast_sample_interfaces::msg::Int64>(
      this, "/my_topic_"+ std::to_string(topic_id), 1,
      std::bind(&MinimalSubscriber::callback, this, _1), agnocast_options);
  }
};

size_t get_num_topics()
{
  rclcpp::Node param_node("param_reader");

  int num_topics = 0;
  param_node.declare_parameter<int>("num_topics", 0);
  param_node.get_parameter("num_topics", num_topics);

  RCLCPP_INFO(param_node.get_logger(), "Using num_topics: %d", num_topics);

  return static_cast<size_t>(num_topics);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const size_t num_topics = get_num_topics();
  if (num_topics == 0) {
    std::cerr << "Invalid num_topics parameter: " << num_topics << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  agnocast::SingleThreadedAgnocastExecutor executor;

  const size_t num_subscribers = MAX_SUBSCRIBER_NUM * num_topics;
  for (size_t i = 0; i < num_subscribers; ++i) {
    executor.add_node(std::make_shared<MinimalSubscriber>());
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
