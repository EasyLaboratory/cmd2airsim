#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include "image_udp_receiver.h"

void rgb_image_callback(const std::vector<uint8_t>& image_data,
                        const ImageMetaData& metadata,
                        image_transport::Publisher& pub) {
  std::cout << "============= RGB Image Callback =============" << std::endl;

  cv::Mat img(metadata.height, metadata.width, CV_8UC3,
              const_cast<uint8_t*>(image_data.data()));
  // 将 RGB 转为 BGR，OpenCV 默认是 BGR 格式
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

  // 转换为 ROS 图像消息
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "base_link";

  // 发布图像消息
  pub.publish(msg);
}

void depth_image_callback(const std::vector<uint8_t>& image_data,
                          const ImageMetaData& metadata,
                          image_transport::Publisher& pub) {
  std::cout << "============= Depth Image Callback =============" << std::endl;

  cv::Mat img(metadata.height, metadata.width, CV_8UC3,
              const_cast<uint8_t*>(image_data.data()));
  // 深度图像通常以 32 位浮点数存储，每个像素代表深度值
  // 转换为 ROS 图像消息
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "base_link";

  // 发布图像消息
  pub.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_udp_receiver_node");
  ros::NodeHandle nh;

  // 创建image_transport对象
  image_transport::ImageTransport it(nh);

  // 创建发布器，分别发布 RGB 和 Depth 数据
  image_transport::Publisher rgb_pub = it.advertise("camera/rgb/image", 1);
  image_transport::Publisher depth_pub = it.advertise("camera/depth/image", 1);

  // 创建UdpReceiver对象，接收RGB图像数据
  UdpReceiver rgb_receiver(50503);    // RGB数据的端口
  UdpReceiver depth_receiver(50504);  // Depth数据的端口

  signal(SIGINT, signal_handler);  // 捕获 Ctrl+C

  // 设置接收和发布数据
  optional<std::chrono::milliseconds> no_sleep_duration;  // 默认构造为空

  // 启动UDP接收线程，并使用回调函数处理接收到的图像数据
  std::thread rgb_thread(&UdpReceiver::start_receiving, &rgb_receiver,
                         no_sleep_duration,
                         std::bind(rgb_image_callback, std::placeholders::_1,
                                   std::placeholders::_2, std::ref(rgb_pub)));

  std::thread depth_thread(
      &UdpReceiver::start_receiving, &depth_receiver, no_sleep_duration,
      std::bind(depth_image_callback, std::placeholders::_1,
                std::placeholders::_2, std::ref(depth_pub)));

  // ROS节点循环
  ros::spin();

  // 等待线程结束
  rgb_thread.join();
  depth_thread.join();

  return 0;
}
