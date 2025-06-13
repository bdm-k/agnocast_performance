#include <chrono>  // std::chrono::*
#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "agnocast/agnocast_ioctl.hpp"  // MAX_PUBLISHER_NUM
#include "agnocast/agnocast_executor.hpp"  // AgnocastExecutor
#include "agnocast/agnocast_multi_threaded_executor.hpp"  // MultiThreadedAgnocastExecutor
#include "agnocast/agnocast_single_threaded_executor.hpp"  // SingleThreadedAgnocastExecutor

#define LOG_EVERY_N 1000

using namespace std::chrono_literals;

static int64_t g_publisher_count = 0;

class MinimalPublisher : public rclcpp::Node
{
  int64_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  int timer_interval_ms_;
  agnocast::Publisher<agnocast_sample_interfaces::msg::Int64>::SharedPtr
    publisher_;

  void timer_callback()
  {
    agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::Int64> message =
      publisher_->borrow_loaned_message();
    message->id = count_;
    publisher_->publish(std::move(message));

    if (++count_ % LOG_EVERY_N == 0) {
      RCLCPP_INFO(this->get_logger(), "publish message: id=%ld", count_);
    }
  }

public:
  explicit MinimalPublisher()
  : Node("minimal_publisher_" + std::to_string(g_publisher_count)), count_(0)
  {
    const int64_t topic_id = g_publisher_count / MAX_PUBLISHER_NUM;

    publisher_ =
      agnocast::create_publisher<agnocast_sample_interfaces::msg::Int64>(
        this, "/my_topic_" + std::to_string(topic_id), 10);

    declare_parameter<int>("timer_interval_ms", 0);
    get_parameter("timer_interval_ms", timer_interval_ms_);

    timer_ = this->create_wall_timer(std::chrono::milliseconds{timer_interval_ms_},
      std::bind(&MinimalPublisher::timer_callback, this));
    timer_->cancel();

    g_publisher_count += 1;
  }

  void reset_timer()
  {
    timer_->reset();
  }
};

struct LaunchParams
{
  int starting_topic_id;
  size_t num_topics;
  bool use_multithreaded_executor;
  int timer_interval_ms;
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

  param_node.declare_parameter<int>("timer_interval_ms", 0);
  param_node.get_parameter("timer_interval_ms", params.timer_interval_ms);

  RCLCPP_INFO(param_node.get_logger(), "Using starting_topic_id: %d", params.starting_topic_id);
  RCLCPP_INFO(param_node.get_logger(), "Using num_topics: %zu", params.num_topics);
  RCLCPP_INFO(param_node.get_logger(), params.use_multithreaded_executor
    ? "Using multi-threaded executor"
    : "Using single-threaded executor");
  RCLCPP_INFO(param_node.get_logger(), "Using timer_interval_ms: %d", params.timer_interval_ms);

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
    std::cerr << "The num_topics parameter should be set\n";
    rclcpp::shutdown();
    return 1;
  }
  if (params.timer_interval_ms == 0) {
    std::cerr << "The timer_interval_ms parameter should be set\n";
    rclcpp::shutdown();
    return 1;
  }

  std::unique_ptr<agnocast::AgnocastExecutor> executor;
  if (params.use_multithreaded_executor) {
    executor = std::make_unique<agnocast::MultiThreadedAgnocastExecutor>();
  } else {
    executor = std::make_unique<agnocast::SingleThreadedAgnocastExecutor>();
  }

  g_publisher_count = params.starting_topic_id * MAX_PUBLISHER_NUM;

  const size_t num_publishers = MAX_PUBLISHER_NUM * params.num_topics;
  std::vector<std::shared_ptr<MinimalPublisher>> publishers = {};
  for (size_t i = 0; i < num_publishers; ++i) {;
    publishers.push_back(std::make_shared<MinimalPublisher>());
    executor->add_node(publishers[i]);
  }

  // Stagger the start time of the timers
  auto interval = 99us / num_publishers;
  for (size_t i = 0; i < num_publishers; ++i) {
    publishers[i]->reset_timer();
    rclcpp::sleep_for(interval);
  }

  executor->spin();

  rclcpp::shutdown();
  return 0;
}
