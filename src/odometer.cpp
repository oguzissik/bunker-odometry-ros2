#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "bunker_msgs/msg/bunker_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Odometer : public rclcpp::Node
{
public:
  Odometer() //initializa node
  : Node("odometer"), x_(0.0), y_(0.0), theta_(0.0), last_time_(0, 0, RCL_ROS_TIME)
  {
    //publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);
    //TF broadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    //subscriber
    sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10,
      std::bind(&Odometer::bunker_callback, this, _1));
    //reset
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&Odometer::reset_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Odometer node started.");
  }

private:
  //bunker callback
  void bunker_callback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  { //checking differences between now and last time stamp
    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    //time can not be negative and when dt>1 robot steps up too much 
    if (dt > 1.0 || dt <= 0.0) return;
    //taking velocities from BunkerStatus
    double vx = msg->linear_velocity;
    double wz = msg->angular_velocity;

    //skid steering kinematics , in each callback integrating a small step
    //if moving forward vx>0 and rotating , theta changes if wz!=0 
    x_ += vx * std::cos(theta_) * dt;
    y_ += vx * std::sin(theta_) * dt;
    theta_ += wz * dt;

    //odom messages,publishing to topic /project_odom
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom"; //world frame
    odom_msg.child_frame_id = "base_link2"; //robot frame
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.angular.z = wz;

    odom_pub_->publish(odom_msg);

    //broadcasting into TF system
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link2";
    //wrt  odom frame where is base_link2
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    //wrt odom frame how did base_link2 rotate? 
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t); //in each callback we publish current position into TF system and  therefore...
                                       //... RViz knows where the robot is 
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // when the service is calles "reset_callback" all becomes 0
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "Odometry reset to zero.");
  }

  double x_, y_, theta_;
  rclcpp::Time last_time_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}
