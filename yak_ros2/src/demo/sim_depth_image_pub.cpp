#include <gl_depth_sim/sim_depth_camera.h>
#include <gl_depth_sim/mesh_loader.h>
#include <gl_depth_sim/interfaces/opencv_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/highgui.hpp>

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
  auto node = rclcpp::Node::make_shared("image_simulator_node");

  // Setup ROS interfaces
  image_transport::ImageTransport it(node);
  image_transport::Publisher image_pub = it.advertise(DEFAULT_IMAGE_TOPIC, 1);
  tf2_ros::TransformBroadcaster broadcaster(node);

  // declare parameters
  node->declare_parameter("mesh");
  node->declare_parameter("world");
  node->declare_parameter("camera");
  node->declare_parameter("orbit_speed");
  node->declare_parameter("framerate");
  node->declare_parameter("radius");
  node->declare_parameter("focal_length");
  node->declare_parameter("z");
  node->declare_parameter("width");
  node->declare_parameter("height");

  // Load ROS parameters
  std::string mesh_path;
  if (!node->get_parameter("mesh", mesh_path))
  {
    RCLCPP_ERROR(node->get_logger(), "User must set the 'mesh' parameter");
    return 1;
  }

  std::string base_frame;
  node->get_parameter_or<std::string>("base_frame", base_frame, "world");

  std::string camera_frame;
  node->get_parameter_or<std::string>("camera_frame", camera_frame, "camera");

  double orbit_speed;
  node->get_parameter_or<double>("orbit_speed", orbit_speed, 1.0);

  double framerate;
  node->get_parameter_or<double>("framerate", framerate, 30.0);

  double radius;
  node->get_parameter_or<double>("radius", radius, 0.5);

  double z;
  node->get_parameter_or<double>("z", z, 0.5);

  double focal_length;
  node->get_parameter_or<double>("focal_length", focal_length, 550.0);

  int width;
  node->get_parameter_or<int>("width", width, 640);

  int height;
  node->get_parameter_or<int>("height", height, 480);

  auto mesh_ptr = gl_depth_sim::loadMesh(mesh_path);

  if (!mesh_ptr)
  {
    RCLCPP_ERROR(node->get_logger(), "Unable to load mesh from path: " + mesh_path);
    return 1;
  }

  gl_depth_sim::CameraProperties props;
  props.width = width;
  props.height = height;
  props.fx = focal_length;
  props.fy = focal_length;
  props.cx = props.width / 2;
  props.cy = props.height / 2;
  props.z_near = 0.25;
  props.z_far = 10.0f;

  // Create the simulation
  gl_depth_sim::SimDepthCamera sim(props);
  sim.add(*mesh_ptr, Eigen::Isometry3d::Identity());

  // In the main (rendering) thread, begin orbiting...
  const auto start = std::chrono::steady_clock::now();

  rclcpp::Rate rate(framerate);

  while (rclcpp::ok())
  {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    Eigen::Vector3d camera_pos(radius * cos(dt * orbit_speed), radius * sin(dt * orbit_speed), z);

    Eigen::Vector3d look_at(0, 0, 0);

    const auto pose = lookat(camera_pos, look_at, Eigen::Vector3d(0, 0, 1));

    const auto depth_img = sim.render(pose);

    cv::Mat cv_img;
    gl_depth_sim::toCvImage16u(depth_img, cv_img);

    cv_bridge::CvImage image;
    image.header.stamp = node->now();
    image.header.frame_id = camera_frame;
    image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    image.image = cv_img;
    image_pub.publish(image.toImageMsg());

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.transform.translation.x = pose.translation().x();
    transform_stamped.transform.translation.y = pose.translation().y();
    transform_stamped.transform.translation.z = pose.translation().z();
    transform_stamped.transform.rotation = tf2::toMsg(Eigen::Quaternion<double>(pose.linear()));

    transform_stamped.header.stamp = image.header.stamp;
    transform_stamped.header.frame_id = base_frame;
    transform_stamped.child_frame_id = camera_frame;
    broadcaster.sendTransform(transform_stamped);

    rate.sleep();
    rclcpp::spin_some(node);
  }

  return 0;
}
