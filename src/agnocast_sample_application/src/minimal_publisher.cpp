#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "agnocast/agnocast_ioctl.hpp"  // MAX_PUBLISHER_NUM

using namespace std::chrono_literals;

static int64_t g_publisher_count = 0;

class MinimalPublisher : public rclcpp::Node
{
  int64_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  agnocast::Publisher<agnocast_sample_interfaces::msg::Int64>::SharedPtr
    publisher_;

  void timer_callback()
  {
    agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::Int64> message =
      publisher_->borrow_loaned_message();

    message->id = count_;

    publisher_->publish(std::move(message));
    RCLCPP_INFO(this->get_logger(), "publish message: id=%ld", count_++);
  }

public:
  explicit MinimalPublisher()
  : Node("minimal_publisher_" + std::to_string(g_publisher_count)), count_(0)
  {
    const int64_t topic_id = g_publisher_count / MAX_PUBLISHER_NUM;

    publisher_ =
      agnocast::create_publisher<agnocast_sample_interfaces::msg::Int64>(
        this, "/my_topic_" + std::to_string(topic_id), 10);

    timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    timer_->cancel();

    g_publisher_count += 1;
  }

  void reset_timer()
  {
    timer_->reset();
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

  const size_t num_publishers = MAX_PUBLISHER_NUM * num_topics;
  std::vector<std::shared_ptr<MinimalPublisher>> publishers = {};
  for (size_t i = 0; i < num_publishers; ++i) {;
    publishers.push_back(std::make_shared<MinimalPublisher>());
    executor.add_node(publishers[i]);
  }

  // Start the timers at staggered intervals
  auto interval = 1000ms / num_publishers;
  for (size_t i = 0; i < num_publishers; ++i) {
    publishers[i]->reset_timer();
    rclcpp::sleep_for(interval);
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
