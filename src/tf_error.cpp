#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "first_project/msg/tf_error_msg.hpp"

using namespace std::chrono_literals;

class TfError : public rclcpp::Node
{
public:
  TfError()
  : Node("tf_error"), travelled_distance_(0.0), last_x_(0.0), last_y_(0.0),
    start_time_(0, 0, RCL_ROS_TIME), started_(false)
    //until first tf_lookup is successfull started_ stays false
  {
    // Listener fills the buffer and we read from buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher for custom error message
    // as requested "publish custom message, type: first_project/tf_error_msg, topic: /tf_error_msg"
    error_pub_ = this->create_publisher<first_project::msg::TfErrorMsg>("/tf_error_msg", 10);

    // Timer starts every 100ms , and we use timer instead of publisher because ...
    //...TF is not a topic we cant subscribe to it , instead we periodically lookup from buffer
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TfError::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "TfError node started.");
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped gt_tf; //ground truth TF read fromg GPS coming from bags
    geometry_msgs::msg::TransformStamped our_tf; // TF that our odometer published

    // Look up ground truth TF from bag: odom -> base_link
    try { //base_link is robot frame Published by GPS , TimePointZero->give latest  transform
      gt_tf = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) { //probably TF still is not published yet
      RCLCPP_WARN(this->get_logger(), "Could not get base_link TF: %s", ex.what());
      return;
    }

    // Look up our computed TF: odom -> base_link2
    try { //base_link2  is  the robot frame we measured to check "dist between base_link and base_link2
      our_tf = tf_buffer_->lookupTransform("odom", "base_link2", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get base_link2 TF: %s", ex.what());
      return;
    }

    // Initialize start time on first successful lookup
    if (!started_) { // !started means first lookup is successfull so  started_=true
      start_time_ = this->get_clock()->now();
      last_x_ = gt_tf.transform.translation.x;
      last_y_ = gt_tf.transform.translation.y;
      started_ = true;
    }

    // Compute Euclidean distance between base_link and base_link2
    // PDF: "compute the distance between base_link and base_link2"
    double dx = gt_tf.transform.translation.x - our_tf.transform.translation.x;
    double dy = gt_tf.transform.translation.y - our_tf.transform.translation.y;
    double error = std::sqrt(dx * dx + dy * dy);

    // Compute travelled distance using ground truth
    // in every timer tick we measure diff between last position and current position measured by GPS...
    //... not measuring it through our odometry since there might be errors
    double step_x = gt_tf.transform.translation.x - last_x_; 
    double step_y = gt_tf.transform.translation.y - last_y_;
    travelled_distance_ += std::sqrt(step_x * step_x + step_y * step_y);
    last_x_ = gt_tf.transform.translation.x;
    last_y_ = gt_tf.transform.translation.y;

    // Compute time from start
    // PDF: "int time_from_start"
    int32_t time_from_start = (int32_t)(this->get_clock()->now() - start_time_).seconds();

    // Fill and publish custom message
    // PDF: "fields: header, float32 tf_error, int time_from_start, float32 travelled_distance"
    first_project::msg::TfErrorMsg msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.tf_error = (float)error;
    msg.time_from_start = time_from_start;
    msg.travelled_distance = (float)travelled_distance_;

    error_pub_->publish(msg);
  }

  // Member variables
  double travelled_distance_;
  double last_x_, last_y_;
  rclcpp::Time start_time_;
  bool started_;
  rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfError>());
  rclcpp::shutdown();
  return 0;
}
