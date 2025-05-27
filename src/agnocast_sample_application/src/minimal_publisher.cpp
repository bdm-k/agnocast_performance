#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "agnocast/agnocast_ioctl.hpp"  // MAX_PUBLISHER_NUM

using namespace std::chrono_literals;

static int64_t publisher_count = 0;

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
  : Node("minimal_publisher_" + std::to_string(publisher_count++)), count_(0)
  {
    publisher_ =
      agnocast::create_publisher<agnocast_sample_interfaces::msg::Int64>(
        this, "/my_topic", 10);

    timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    timer_->cancel();
  }

  void reset_timer()
  {
    timer_->reset();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  agnocast::SingleThreadedAgnocastExecutor executor;
  std::shared_ptr<MinimalPublisher> publishers[MAX_PUBLISHER_NUM];
  for (int64_t i = 0; i < MAX_PUBLISHER_NUM; ++i) {
    publishers[i] = std::make_shared<MinimalPublisher>();
    executor.add_node(publishers[i]);
  }

  // Start the timers at staggered intervals
  auto interval = 1000ms / MAX_PUBLISHER_NUM;
  for (int64_t i = 0; i < MAX_PUBLISHER_NUM; ++i) {
    publishers[i]->reset_timer();
    rclcpp::sleep_for(interval);
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
