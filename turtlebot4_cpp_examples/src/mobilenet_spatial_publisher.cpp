/*
 * Copyright 2022 Clearpath Robotics, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>

#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>
#include <depthai_bridge/SpatialDetectionConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

dai::Pipeline createPipeline(bool syncNN, std::string nnPath, double fps)
{
  dai::Pipeline pipeline;
  auto colorCam = pipeline.create<dai::node::ColorCamera>();
  auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
  auto spatialDetectionNetwork = pipeline.create<dai::node::MobileNetSpatialDetectionNetwork>();
  auto nnOut = pipeline.create<dai::node::XLinkOut>();

  dai::node::MonoCamera::Properties::SensorResolution monoResolution;
  auto monoLeft = pipeline.create<dai::node::MonoCamera>();
  auto monoRight = pipeline.create<dai::node::MonoCamera>();
  auto stereo = pipeline.create<dai::node::StereoDepth>();
  auto xoutBoundingBoxDepthMapping = pipeline.create<dai::node::XLinkOut>();
  auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

  auto logger = pipeline.create<dai::node::SystemLogger>();
  logger->setRate(1.0f);
  auto xout = pipeline.create<dai::node::XLinkOut>();
  xout->setStreamName("sysInfo");
  logger->out.link(xout->input);

  xlinkOut->setStreamName("preview");
  nnOut->setStreamName("detections");
  xoutBoundingBoxDepthMapping->setStreamName("boundingBoxDepthMapping");
  xoutDepth->setStreamName("depth");

  xlinkOut->setFpsLimit(fps);
  nnOut->setFpsLimit(fps);
  xoutBoundingBoxDepthMapping->setFpsLimit(fps);
  xoutDepth->setFpsLimit(fps);

  colorCam->setPreviewSize(300, 300);
  colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  colorCam->setInterleaved(false);
  colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  colorCam->setFps(fps);

  monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;

  // MonoCamera
  monoLeft->setResolution(monoResolution);
  monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
  monoRight->setResolution(monoResolution);
  monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

  // StereoDepth
  stereo->initialConfig.setConfidenceThreshold(200);
  stereo->setRectifyEdgeFillColor(0);   // black, to better see the cutout
  stereo->initialConfig.setLeftRightCheckThreshold(5);
  stereo->setLeftRightCheck(true);
  stereo->setExtendedDisparity(false);
  stereo->setSubpixel(false);

  spatialDetectionNetwork->setBlobPath(nnPath);
  spatialDetectionNetwork->setConfidenceThreshold(0.5f);
  spatialDetectionNetwork->input.setBlocking(false);
  spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
  spatialDetectionNetwork->setDepthLowerThreshold(100);
  spatialDetectionNetwork->setDepthUpperThreshold(5000);

  // Link plugins CAM -> STEREO -> XLINK
  monoLeft->out.link(stereo->left);
  monoRight->out.link(stereo->right);

  // Link plugins CAM -> NN -> XLINK
  colorCam->preview.link(spatialDetectionNetwork->input);
  if (syncNN) {
    spatialDetectionNetwork->passthrough.link(xlinkOut->input);
  } else {
    colorCam->preview.link(xlinkOut->input);
  }

  spatialDetectionNetwork->out.link(nnOut->input);
  spatialDetectionNetwork->boundingBoxMapping.link(xoutBoundingBoxDepthMapping->input);
  stereo->depth.link(spatialDetectionNetwork->inputDepth);
  spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

  return pipeline;
}

void printSystemInformation(dai::SystemInformation info) {
    printf("Ddr used / total - %.2f / %.2f MiB\n", info.ddrMemoryUsage.used / (1024.0f * 1024.0f), info.ddrMemoryUsage.total / (1024.0f * 1024.0f));
    printf("Cmx used / total - %.2f / %.2f MiB\n", info.cmxMemoryUsage.used / (1024.0f * 1024.0f), info.cmxMemoryUsage.total / (1024.0f * 1024.0f));
    printf("LeonCss heap used / total - %.2f / %.2f MiB\n",
           info.leonCssMemoryUsage.used / (1024.0f * 1024.0f),
           info.leonCssMemoryUsage.total / (1024.0f * 1024.0f));
    printf("LeonMss heap used / total - %.2f / %.2f MiB\n",
           info.leonMssMemoryUsage.used / (1024.0f * 1024.0f),
           info.leonMssMemoryUsage.total / (1024.0f * 1024.0f));
    const auto& t = info.chipTemperature;
    printf("Chip temperature - average: %.2f, css: %.2f, mss: %.2f, upa: %.2f, dss: %.2f\n", t.average, t.css, t.mss, t.upa, t.dss);
    printf("Cpu usage - Leon CSS: %.2f %%, Leon MSS: %.2f %%\n", info.leonCssCpuUsage.average * 100, info.leonMssCpuUsage.average * 100);
    printf("----------------------------------------\n");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mobilenet_spatial_node");

  std::string tfPrefix;
  std::string cameraParamUri = "package://turtlebot4_cpp_examples/params/camera";
  std::string nnPath(BLOB_PATH);
  bool syncNN;
  int bad_params = 0;
  double fps;

  node->declare_parameter("tf_prefix", "oak");
  node->declare_parameter("camera_param_uri", cameraParamUri);
  node->declare_parameter("sync_nn", true);
  node->declare_parameter<std::string>("nn_path", "");
  node->declare_parameter("fps", 15.0);
  node->get_parameter("tf_prefix", tfPrefix);
  node->get_parameter("camera_param_uri", cameraParamUri);
  node->get_parameter("sync_nn", syncNN);
  node->get_parameter("fps", fps);

  // Uses the path from param if passed or else uses from BLOB_PATH from CMAKE
  std::string nnParam;
  node->get_parameter("nn_path", nnParam);
  if (!nnParam.empty()) {
    node->get_parameter("nn_path", nnPath);
  }

  dai::Pipeline pipeline = createPipeline(syncNN, nnPath, fps);
  dai::Device device(pipeline);

  auto qSysInfo = device.getOutputQueue("sysInfo", 4, false);

  std::shared_ptr<dai::DataOutputQueue> previewQueue = device.getOutputQueue(
    "preview", 30, false);
  std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = device.getOutputQueue(
    "detections", 30, false);
  std::shared_ptr<dai::DataOutputQueue> depthQueue = device.getOutputQueue(
    "depth", 4, false);
  std::shared_ptr<dai::DataOutputQueue> bboxQueue = device.getOutputQueue(
    "boundingBoxDepthMapping", 4, false);

  std::string color_uri = cameraParamUri + "/" + "color.yaml";

  dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
  dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(previewQueue,
    node,
    std::string("color/image"),
    std::bind(
      &dai::rosBridge::ImageConverter::toRosMsg,
      &rgbConverter,
      std::placeholders::_1,
      std::placeholders::_2),
    15,
    color_uri,
    "color");


  dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame",
    300, 300, false);
  dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray,
    dai::SpatialImgDetections> detectionPublish(nNetDataQueue,
    node,
    std::string("color/mobilenet_spatial_detections"),
    std::bind(
      &dai::rosBridge::SpatialDetectionConverter::toRosMsg,
      &detConverter,
      std::placeholders::_1,
      std::placeholders::_2),
    15);

  // addPublisherCallback works only when the dataqueue is non blocking.
  detectionPublish.addPublisherCallback();
  rgbPublish.addPublisherCallback();

  while(true)
  {
    rclcpp::spin_some(node);
    auto sysInfo = qSysInfo->tryGet<dai::SystemInformation>();
    if (sysInfo != nullptr)
    {
        printSystemInformation(*sysInfo);
    }
  }
  

  return 0;
}
