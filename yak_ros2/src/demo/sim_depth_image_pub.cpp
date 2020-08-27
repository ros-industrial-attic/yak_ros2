#include <gl_depth_sim/sim_depth_camera.h>
#include <gl_depth_sim/mesh_loader.h>
#include <gl_depth_sim/interfaces/opencv_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/highgui.hpp>

#include <chrono>

const static std::string DEFAULT_IMAGE_TOPIC = "image";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("image_simulator_node");

  // Setup ROS interfaces
  image_transport::ImageTransport it(node);
  image_transport::Publisher image_pub = it.advertise(DEFAULT_IMAGE_TOPIC, 1);
  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(buffer);

  // declare parameters
  node->declare_parameter("mesh");
  node->declare_parameter("base_frame");
  node->declare_parameter("camera_frame");
  node->declare_parameter("framerate");
  node->declare_parameter("focal_length");
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

  double framerate;
  node->get_parameter_or<double>("framerate", framerate, 30.0);

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
  rclcpp::Rate rate(framerate);

  while (rclcpp::ok())
  {
    rate.sleep();
    try
    {
      geometry_msgs::msg::TransformStamped tf_camera_st = buffer.lookupTransform(base_frame, camera_frame, tf2::TimePointZero);
      const auto pose = tf2::transformToEigen(tf_camera_st);
      const auto depth_img = sim.render(pose);

      cv::Mat cv_img;
      gl_depth_sim::toCvImage16u(depth_img, cv_img);

      cv_bridge::CvImage image;
      image.header.stamp = tf_camera_st.header.stamp;
      image.header.frame_id = tf_camera_st.child_frame_id;
      image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      image.image = cv_img;
      image_pub.publish(image.toImageMsg());
    }
    catch (tf2::LookupException &e)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
    }

    rclcpp::spin_some(node);
  }

  return 0;
}
