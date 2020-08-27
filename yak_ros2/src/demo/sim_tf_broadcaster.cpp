#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <chrono>

const static std::string DEFAULT_IMAGE_TOPIC = "image";

static Eigen::Isometry3d lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up)
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Isometry3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sim_tf_broadcaster_node");

  // Setup ROS interfaces
  tf2_ros::TransformBroadcaster broadcaster(node);

  // declare parameters
  node->declare_parameter("base_frame");
  node->declare_parameter("camera_frame");
  node->declare_parameter("orbit_speed");
  node->declare_parameter("hz");
  node->declare_parameter("radius");
  node->declare_parameter("z");

  std::string base_frame;
  node->get_parameter_or<std::string>("base_frame", base_frame, "world");

  std::string camera_frame;
  node->get_parameter_or<std::string>("camera_frame", camera_frame, "camera");

  double orbit_speed;
  node->get_parameter_or<double>("orbit_speed", orbit_speed, 1.0);

  double hz;
  node->get_parameter_or<double>("hz", hz, 30.0);

  double radius;
  node->get_parameter_or<double>("radius", radius, 0.5);

  double z;
  node->get_parameter_or<double>("z", z, 0.5);

  const auto start = std::chrono::steady_clock::now();

  rclcpp::Rate rate(hz);

  while (rclcpp::ok())
  {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    Eigen::Vector3d camera_pos(radius * cos(dt * orbit_speed), radius * sin(dt * orbit_speed), z);

    Eigen::Vector3d look_at(0, 0, 0);

    const auto pose = lookat(camera_pos, look_at, Eigen::Vector3d(0, 0, 1));

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.transform.translation.x = pose.translation().x();
    transform_stamped.transform.translation.y = pose.translation().y();
    transform_stamped.transform.translation.z = pose.translation().z();
    transform_stamped.transform.rotation = tf2::toMsg(Eigen::Quaternion<double>(pose.linear()));

    transform_stamped.header.stamp = node->now();
    transform_stamped.header.frame_id = base_frame;
    transform_stamped.child_frame_id = camera_frame;
    broadcaster.sendTransform(transform_stamped);

    rate.sleep();
    rclcpp::spin_some(node);
  }

  return 0;
}
