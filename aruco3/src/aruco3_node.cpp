

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// openCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// openCV ArUco Tracker
#include "aruco/aruco.h"

using namespace std;

// Topics for the data
static string topicInImage, topicOutOdom, topicOutPixel, topicCameraInfo;
static string parentFrameId;

// Subs - Pubs
static ros::Subscriber imageSub;
static ros::Subscriber cameraInfoSub;
static std::vector<ros::Publisher> pubOdom;
static std::vector<ros::Publisher> pubPixel;
static int pubPixelMode = 0;

const int objectnum = 64;  // This is the amount of maximum possible marker IDs

// Gui Options
static int gui = 0;
static int drawCube = 0;
static int drawAxis = 0;
static int fps = 30;
static string windowName = "ArUco3 Tracker";

// Image
static cv::Mat image;
static std_msgs::Header imageHeader;

// Dictionary
static string dictionary_type = "ARUCO";
static aruco::Dictionary dictionary;

// CameraParameter
static aruco::CameraParameters CamParam;
static sensor_msgs::CameraInfo cameraInfo;
static volatile bool setCameraInfo = false;

// MarkerDetector
static aruco::MarkerDetector MDetector;
// static std::map<uint32_t, aruco::MarkerPoseTracker> MTracker;
static int markerSize = 100; //in mm
static int show_threshold;
static std::string configFile("");
static float error_correction_rate = 0.0f;
static int threshold = 11;

// Verbosity
static int verbose = 0;

// Prototypes
void initArucoParams();
void mainLoopTracking();
void callbackCameraInfo(sensor_msgs::CameraInfo msg);
void callbackImage(sensor_msgs::ImageConstPtr msg);
static void programOptions(ros::NodeHandle &n);

void mainLoopTracking() {
  std::map<uint32_t, aruco::MarkerPoseTracker> MTracker;
  // Ok, let's detect
  ros::Time t1 = ros::Time::now();
  std::vector<aruco::Marker> Markers = MDetector.detect(image, CamParam, markerSize, false);
  ros::Time t2 = ros::Time::now();
  ros::Duration d = t2 - t1;
  if (verbose)
    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] " << "time fid:" << d << "[s]");

  if (gui && show_threshold) {
    cv::Mat thresh_img = MDetector.getThresholdedImage();
    cv::imshow("thresh", thresh_img);
  }
  for (auto &marker:Markers) {//for each marker
    bool pose_bool = MTracker[marker.id].estimatePose(marker, CamParam, markerSize, 1.0); //call its tracker and estimate the pose // 1.0 is the minErrorRation
    if (verbose) {
      ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]" << marker);
    }
  }
  if (gui) {
    // for each marker, draw info and its boundaries in the image
    for (auto &Marker : Markers) {
      Marker.draw(image, cv::Scalar(0, 0, 255), 2, true);
    }
    // draw a 3d cube in each marker if there is 3d info
    if (CamParam.isValid() && markerSize != -1) {
      for (auto &Marker : Markers) {
        if (drawCube)
          aruco::CvDrawingUtils::draw3dCube(image, Marker, CamParam);
        if (drawAxis)
          aruco::CvDrawingUtils::draw3dAxis(image, Marker, CamParam);
      }
    }
    if (show_threshold) {
      cv::Mat thresh_img = MDetector.getThresholdedImage();
      cv::imshow(windowName + "_threshold", thresh_img);
    }
    cv::imshow(windowName, image);
    cv::waitKey(1);
  }

  for (auto &Marker : Markers) {

    // Get the full 4x4 transformation (Note: translation is in mm)
    cv::Mat rtMatrix = MTracker[Marker.id].getRTMatrix();

    // Print the transformation matrix
    std::stringstream ss;
    ss << "rtMatrix of Marker " << Marker.id << "\n";
    for (int y = 0; y < 4; y++) {
      for (int x = 0; x < 4; x++) {
        const float value = rtMatrix.at<float>(x + 4 * y);
        ss << value << "\t";
      }
      ss << std::endl;

    }
    ROS_DEBUG_STREAM(ss.str());

    // Define the the message header
    nav_msgs::Odometry odom;
    odom.header = imageHeader;
    odom.child_frame_id = std::string("base_link/") + std::to_string(Marker.id);

    // Set the rotation
    tf::Quaternion quat;
    geometry_msgs::Quaternion quatMsg;
    const tf::Matrix3x3 R(rtMatrix.at<float>(0), rtMatrix.at<float>(1), rtMatrix.at<float>(2),
                          rtMatrix.at<float>(4), rtMatrix.at<float>(5), rtMatrix.at<float>(6),
                          rtMatrix.at<float>(8), rtMatrix.at<float>(9), rtMatrix.at<float>(10));
    R.getRotation(quat);
    tf::quaternionTFToMsg(quat, quatMsg);
    odom.pose.pose.orientation = quatMsg;
    // Set the translation
    odom.pose.pose.position.x = rtMatrix.at<float>(3) / 1000.0;
    odom.pose.pose.position.y = rtMatrix.at<float>(7) / 1000.0;
    odom.pose.pose.position.z = rtMatrix.at<float>(11) / 1000.0;

    // Set fixed covariances
    const boost::array<double, 36> covariance = {{
                                                     .1, 0, 0, 0, 0, 0,
                                                     0, .1, 0, 0, 0, 0,
                                                     0, 0, .15, 0, 0, 0,
                                                     0, 0, 0, .01, 0, 0,
                                                     0, 0, 0, 0, .01, 0,
                                                     0, 0, 0, 0, 0, .01}};
    odom.pose.covariance = covariance;

    // Publish
    pubOdom.at(Marker.id).publish(odom);

    if (pubPixelMode) {
      nav_msgs::Odometry pixel = odom;
      pixel.header = imageHeader;
      pixel.child_frame_id = std::string("base_link/") + std::to_string(Marker.id);
      pixel.pose.pose.position.x = Marker.getCenter().x;
      pixel.pose.pose.position.y = Marker.getCenter().y;
      pixel.pose.pose.position.z = 0;
      pubPixel.at(Marker.id).publish(pixel);
    }
  }
}

void initArucoParams() {
  /*
   * Inspired by aruco_ros_tracker
   * Source: https://github.com/pal-robotics/aruco_ros/blob/indigo-devel/aruco_ros/src/aruco_ros_utils.cpp
   */
  cv::Mat cameraMatrix(cv::Size(3, 3), CV_64FC1);
  cv::Mat distorsionCoeff(cv::Size(1, (uint) cameraInfo.D.size()), CV_64FC1);

  for (int i = 0; i < cameraMatrix.total(); i++)
    cameraMatrix.at<double>(i % 3, i - (i % 3) * 3) = cameraInfo.K[i];


  for (int i = 0; i < cameraInfo.D.size(); i++) {
    distorsionCoeff.at<double>(0, i) = cameraInfo.D[i];
  }

  ROS_INFO_STREAM("[" << ros::this_node::getName() << "]" << " cameraMatrix: " << cameraMatrix);
  ROS_INFO_STREAM("[" << ros::this_node::getName() << "]" << " distorsionCoeff: " << distorsionCoeff);

  CamParam.setParams(cameraMatrix, distorsionCoeff, cv::Size(cameraInfo.height, cameraInfo.width));

  if (!configFile.empty()) {
    MDetector.loadParamsFromFile(configFile);
  } else {
    if (dictionary_type == "ARUCO_MIP_36h12" || dictionary_type == "ARUCO" || dictionary_type == "ARUCO_MIP_25h7"
        || dictionary_type == "ARUCO_MIP_16h3" || dictionary_type == "ARTAG" || dictionary_type == "ARTOOLKITPLUS"
        || dictionary_type == "ARTOOLKITPLUSBCH" || dictionary_type == "TAG16h5" || dictionary_type == "TAG25h7"
        || dictionary_type == "TAG25h9" || dictionary_type == "TAG36h11" || dictionary_type == "TAG36h10"
        || dictionary_type == "CHILITAGS" || dictionary_type == "ALL_DICTS") {
      MDetector.setDictionary(aruco::Dictionary::getTypeFromString(dictionary_type), error_correction_rate);
    } else {
      MDetector.setDictionary(dictionary_type, error_correction_rate);
    }
    // Note: These are very conservative parameters to get the best results out of the tracker
    MDetector.getParameters().detectEnclosedMarkers(false);
    MDetector.getParameters().setDetectionMode(aruco::DetectionMode::DM_NORMAL, 0.f);
//    MDetector.getParameters().setDetectionMode(aruco::DetectionMode::DM_FAST, 0.f);
//    MDetector.getParameters().setDetectionMode(aruco::DetectionMode::DM_VIDEO_FAST, 0.f);
    MDetector.getParameters().setCornerRefinementMethod(aruco::CornerRefinementMethod::CORNER_LINES);
    MDetector.getParameters().ThresHold = threshold;
  }
  setCameraInfo = true;
}

static void programOptions(ros::NodeHandle &n) {
  n.param<std::string>("topic_in_image", topicInImage, "/genicam/cam4"); // Video parameter for the camera
  n.param<std::string>("image_camera_info", topicCameraInfo, "/genicam/camera_info"); // Camera info for the image
  n.param<std::string>("topic_out_odom", topicOutOdom, "/odom"); // scope for sending the odometries
  n.param<std::string>("topic_out_pixel", topicOutPixel, "/pixel"); // scope for sending the pixel data
  n.param<int>("gui", gui, 0); // Show the rectified image by OpenCV
  n.param<int>("draw_cube", drawCube, 0); // Draw cubes on detected marker
  n.param<int>("draw_axis", drawAxis, 0); // Draw axis on detected marker
  n.param<std::string>("window_name", windowName, ros::this_node::getName()); // Window Name
  n.param<int>("marker_size", markerSize, 100); // Marker Size in mm
  n.param<std::string>("dictionary", dictionary_type, "ARUCO"); // Default Dictionarytype: ARUCO,ARUCO_MIP_36h12 or Path to custom dictionary .dict-File
  n.param<std::string>("config_file", configFile, ""); // Default config which overwrites all set parameters
  n.param<int>("verbose", verbose, 0); // Verbosity
  n.param<int>("fps", fps, 15); // FPS
  n.param<int>("show_threshold", show_threshold, 0); // Show Threshhold
  n.param<int>("pub_pixel", pubPixelMode, 0);
  n.param<int>("threshold", threshold, 11);
  n.param<float>("error_correction_rate", error_correction_rate, 0.0f); // error_correction_rate value indicating the correction error allowed. Is in range [0,1]. 0 means no correction at all.
  // So an erroneous bit will result in discarding the marker. 1, mean full correction. The maximum number of bits that can be corrected depends on each ditionary. We recommend using values from 0 to 0.5. (in general, this will allow up to 3 bits or correction).
}

void callbackCameraInfo(sensor_msgs::CameraInfo msg) {
  if (cameraInfo.K != msg.K) { // if something changed
    cameraInfo = msg;
    initArucoParams();
    ROS_INFO("[%s] Update camera_info.", ros::this_node::getName().c_str());
  }
}

void callbackImage(sensor_msgs::ImageConstPtr msg) {
  if (!setCameraInfo) {
    ROS_WARN("[%s] Tracking abort, no cameraInfo recieved yet.", ros::this_node::getName().c_str());
    return;
  }
  if (msg->encoding == "mono8") {
    cv::cvtColor(cv_bridge::toCvShare(msg, msg->encoding)->image, image, cv::COLOR_GRAY2BGR);
  } else if (msg->encoding == "rgb8") {
    image = cv_bridge::toCvShare(msg, msg->encoding)->image;
  } else {
    ROS_WARN("[%s] Unknown Image encoding %s.", ros::this_node::getName().c_str(), msg->encoding.c_str());
    return;
  }
  imageHeader = msg->header;
  // Sharpen image
  // cv::Mat frame;
  // cv::GaussianBlur(image, frame, cv::Size(0, 0), 3);
  // cv::addWeighted(image, 1.5, frame, -0.5, 0, image);
  // Track
  mainLoopTracking();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle n("~");
  programOptions(n);

  imageSub = n.subscribe(topicInImage, 1, callbackImage);
  cameraInfoSub = n.subscribe(topicCameraInfo, 1, callbackCameraInfo);

  // Allocate the publisher for the maximum number of markers
  pubOdom.resize(objectnum);
  pubPixel.resize(objectnum);
  int idx = 0;
  for (auto it = pubOdom.begin(); it != pubOdom.end(); ++it, ++idx) {
    std::stringstream ss;
    ss << n.getNamespace() << std::string("/") << topicOutOdom << std::string("/") << idx;
    *it = n.advertise<nav_msgs::Odometry>(ss.str(), 1);
  }
  idx = 0;
  for (auto it = pubPixel.begin(); it != pubPixel.end(); ++it, ++idx) {
    std::stringstream ss;
    ss << n.getNamespace() << std::string("/") << topicOutPixel << std::string("/") << idx;
    *it = n.advertise<nav_msgs::Odometry>(ss.str(), 1);
  }

  if (gui) {
    cv::namedWindow(windowName, CV_WINDOW_FREERATIO);
    if (show_threshold)
      cv::namedWindow(windowName + "_threshold", CV_WINDOW_FREERATIO);
  }
  ros::Rate rate(fps);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
