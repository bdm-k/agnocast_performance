#include <thread>
#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "agnocast/agnocast_ioctl.hpp"  // MAX_SUBSCRIBER_NUM
#include "agnocast/agnocast_executor.hpp"  // AgnocastExecutor
#include "agnocast/agnocast_multi_threaded_executor.hpp"  // MultiThreadedAgnocastExecutor
#include "agnocast/agnocast_single_threaded_executor.hpp"  // SingleThreadedAgnocastExecutor

#define LOG_EVERY_N 4000

using std::placeholders::_1;

static int64_t g_subscriber_count = 0;

class MinimalSubscriber : public rclcpp::Node
{
  agnocast::Subscription<agnocast_sample_interfaces::msg::Int64>::SharedPtr sub_;

  void callback(
    const agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::Int64> & message)
  {
    if (message->id % LOG_EVERY_N == 0) {
      RCLCPP_INFO(this->get_logger(), "subscribe message: id=%ld", message->id);
    }
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

struct LaunchParams
{
  int starting_topic_id;
  size_t num_topics;
  bool use_multithreaded_executor;
  size_t ros2_thread_count;
  size_t agnocast_thread_count;
};

LaunchParams get_launch_params()
{
  rclcpp::Node param_node("param_reader");

  LaunchParams params;

  param_node.declare_parameter<int>("starting_topic_id", -1);
  param_node.get_parameter("starting_topic_id", params.starting_topic_id);

  int num_topics = 0;
  param_node.declare_parameter<int>("num_topics", 0);
  param_node.get_parameter("num_topics", num_topics);
  params.num_topics = static_cast<size_t>(num_topics);

  param_node.declare_parameter<bool>("use_multithreaded_executor", false);
  param_node.get_parameter("use_multithreaded_executor", params.use_multithreaded_executor);

  int ros2_thread_count = 0;
  param_node.declare_parameter<int>("ros2_thread_count", 0);
  param_node.get_parameter("ros2_thread_count", ros2_thread_count);
  params.ros2_thread_count = static_cast<size_t>(ros2_thread_count);

  int agnocast_thread_count = 0;
  param_node.declare_parameter<int>("agnocast_thread_count", 0);
  param_node.get_parameter("agnocast_thread_count", agnocast_thread_count);
  params.agnocast_thread_count = static_cast<size_t>(agnocast_thread_count);

  RCLCPP_INFO(param_node.get_logger(), "Using starting_topic_id: %d", params.starting_topic_id);
  RCLCPP_INFO(param_node.get_logger(), "Using num_topics: %zu", params.num_topics);
  RCLCPP_INFO(param_node.get_logger(), params.use_multithreaded_executor
    ? "Using multi-threaded executor"
    : "Using single-threaded executor");
  if (params.use_multithreaded_executor) {
    RCLCPP_INFO(param_node.get_logger(), "Using ros2_thread_count %zu",
      params.ros2_thread_count == 0
        ? std::thread::hardware_concurrency() / 2
        : params.ros2_thread_count);
    RCLCPP_INFO(param_node.get_logger(), "Using agnocast_thread_count %zu",
      params.agnocast_thread_count == 0
        ? std::thread::hardware_concurrency() / 2
        : params.agnocast_thread_count);
  }

  return params;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const LaunchParams params = get_launch_params();
  if (params.starting_topic_id == -1) {
    std::cerr << "The starting_topic_id parameter should be set\n";
    rclcpp::shutdown();
    return 1;
  }
  if (params.num_topics == 0) {
    std::cerr << "The num_topics parameter should not be 0\n";
    rclcpp::shutdown();
    return 1;
  }
  if (params.num_topics >= 16) {
    std::cerr <<
      "The num_topics should be less than 16. "
      "Otherwise, the number of mqueues will exceed the limit.\n";
    rclcpp::shutdown();
    return 1;
  }

  std::unique_ptr<agnocast::AgnocastExecutor> executor;
  if (params.use_multithreaded_executor) {
    executor = std::make_unique<agnocast::MultiThreadedAgnocastExecutor>(
      rclcpp::ExecutorOptions(), params.ros2_thread_count, params.agnocast_thread_count);
  } else {
    executor = std::make_unique<agnocast::SingleThreadedAgnocastExecutor>();
  }

  g_subscriber_count = params.starting_topic_id * MAX_SUBSCRIBER_NUM;

  const size_t num_subscribers = MAX_SUBSCRIBER_NUM * params.num_topics;
  for (size_t i = 0; i < num_subscribers; ++i) {
    executor->add_node(std::make_shared<MinimalSubscriber>());
  }

  executor->spin();

  rclcpp::shutdown();
  return 0;
}
