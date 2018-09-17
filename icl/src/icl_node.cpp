// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// ICL
#include <ICLQt/Common.h>
#include <ICLCore/OpenCV.h>
#include <ICLMarkers/FiducialDetector.h>
#include <ICLMarkers/FiducialDetectorPluginForQuads.h>
#include <ICLGeom/Scene.h>
#include <ICLGeom/ComplexCoordinateFrameSceneObject.h>

// Boost
#include <boost/bimap.hpp>

#include <cmath>

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

// CameraParameter
static sensor_msgs::CameraInfo cameraInfo;
static volatile bool setCameraInfo = false;
static int focalLength = 8; // in mm

// Gui Options
static string windowName = "ICL-CV Tracker";
static int fps = 15;
static HSplit iclGui;
static icl::geom::Scene scene;
static int draw3dAxis = 0;
static DrawHandle draw;

static int verbose = 0;

// Tracker
static icl::markers::FiducialDetector *fid = 0;
static string markerType = "bch";
static int markerSize = 100; //in mm
static string fidConfigPath = "";
static int objectnum = 64;  // This is the amount of maximum possible marker IDs
const string objectnumString = "[0-" + std::to_string(objectnum) + "]";

// Icl1 Valid Marker
typedef boost::bimap<int, int> bm_type;
bm_type iclValidMarker;

// Image
static int monocolor = 1;
static ::cv::Mat image;
static std_msgs::Header imageHeader;

// Prototypes
void mainLoopTracking();
void callbackCameraInfo(sensor_msgs::CameraInfo msg);
void callbackImage(sensor_msgs::ImageConstPtr msg);
void initIclParameter();
void initBoostBimap();
static void programOptions(ros::NodeHandle &n);
void run();
void publishMarker2d(const int &id, const utils::Point32f center, const geom::Vec &rot3d);
void publishMarker3d(const int &id, const geom::Vec &trans3d, const geom::Vec &rot3d);

void mainLoopTracking() {

//  ::cv::Mat temp;
//  ::cv::resize(image, temp, ::cv::Size(image.size().width/2, image.size().height/2));
  icl::core::ImgBase *image_icl = icl::core::mat_to_img(&image);

  ros::Time t1 = ros::Time::now();
  const std::vector<Fiducial> &fids = fid->detect(image_icl);
  ros::Time t2 = ros::Time::now();
  ros::Duration d = t2 - t1;

  if (monocolor)
    ::cv::cvtColor(image, image, CV_GRAY2BGR);
  image_icl = icl::core::mat_to_img(&image);

  if (verbose)
    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] " << "time fid:" << d << "[s] \t detected marker:" << fids.size());

  draw = image_icl;
  draw->linewidth(2);

  for (unsigned int i = 0; i < fids.size(); i++) {
    const int id = stoi(fids[i].getName());

    if (markerType == "icl1" && iclValidMarker.right.find(id)->second >= objectnum) {
      continue;

    } else if (markerType != "icl1" && id >= objectnum)
      continue;

    utils::Point32f c = fids[i].getCenter2D();
    float rot = fids[i].getRotation2D();

    const std::vector<Fiducial::KeyPoint> &kps = fids[i].getKeyPoints2D();
    int n = (int) kps.size();

    if (n >= 4) {
      geom::Vec trans3d = fids[i].getCenter3D();
      geom::Vec rot3d = fids[i].getRotation3D();

      if (pubPixelMode)
        publishMarker2d(id, c, rot3d);

      if (verbose) {
        ROS_INFO_STREAM("[" << ros::this_node::getName() << "] " << "id: " << id << " = "
                            << "(" << trans3d.x << ", " << trans3d.y << ", " << trans3d.z << ")"
                            << "(" << rot3d.x << ", " << rot3d.y << ", " << rot3d.z << ")");
        publishMarker3d(id, trans3d, rot3d);
      }

      if (draw3dAxis) {
        draw->color(255, 0, 0, 255);
        draw->arrow(c, c + utils::Point32f(cos(rot3d.x), sin(rot3d.x)) * 100);

        draw->color(0, 0, 255, 255);
        draw->arrow(c, c + utils::Point32f(cos(rot3d.y), sin(rot3d.y)) * 100);

        draw->color(0, 255, 0, 255);
        draw->arrow(c, c + utils::Point32f(cos(rot3d.z), sin(rot3d.z)) * 100);
      }
    }
    // Marker Text Id
    draw->color(0, 0, 255, 255);
    draw->text(fids[i].getName(), c.x, c.y, 20);
    // Marker Orientationarrow
    draw->color(100, 255, 0, 255);
    draw->arrow(c, c + utils::Point32f(cos(rot), sin(rot)) * 100);
    // Marker Area
    draw->color(100, 255, 0, 255);
    draw->linestrip(fids[i].getCorners2D());

  }
}

void publishMarker2d(const int &id, const utils::Point32f center, const geom::Vec &rot3d) {
  // euler to quat
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(rot3d.x, rot3d.y, rot3d.z);
  nav_msgs::Odometry pixel;
  pixel.header = imageHeader;
  pixel.child_frame_id = std::string("base_link/") + std::to_string(id);
  pixel.pose.pose.position.x = center.x;
  pixel.pose.pose.position.y = center.y;
  pixel.pose.pose.position.z = 0;
  pixel.pose.pose.position.z = 0;
  pixel.pose.pose.orientation.x = quaternion.getX();
  pixel.pose.pose.orientation.y = quaternion.getY();
  pixel.pose.pose.orientation.z = quaternion.getZ();
  pixel.pose.pose.orientation.w = quaternion.getW();

  if (markerType == "icl1")
    pubPixel.at(iclValidMarker.right.find(id)->second).publish(pixel);
  else
    pubPixel.at(id).publish(pixel);
}

void publishMarker3d(const int &id, const geom::Vec &trans3d, const geom::Vec &rot3d) {

  // euler to quat
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(rot3d.x, rot3d.y, rot3d.z);

  nav_msgs::Odometry odom;
  odom.header = imageHeader;
  odom.child_frame_id = std::string("base_link/") + std::to_string(id);
  odom.pose.pose.position.x = trans3d.x / 1000.0;
  odom.pose.pose.position.y = trans3d.y / 1000.0;
  odom.pose.pose.position.z = trans3d.z / 1000.0;
  odom.pose.pose.orientation.x = quaternion.getX();
  odom.pose.pose.orientation.y = quaternion.getY();
  odom.pose.pose.orientation.z = quaternion.getZ();
  odom.pose.pose.orientation.w = quaternion.getW();
  const boost::array<double, 36> covariance = {{
                                                 .1, 0, 0, 0, 0, 0,
                                                 0, .1, 0, 0, 0, 0,
                                                 0, 0, .15, 0, 0, 0,
                                                 0, 0, 0, .01, 0, 0,
                                                 0, 0, 0, 0, .01, 0,
                                                 0, 0, 0, 0, 0, .01}};
  odom.pose.covariance = covariance;

  if (markerType == "icl1")
    pubOdom.at(iclValidMarker.right.find(id)->second).publish(odom);
  else
    pubOdom.at(id).publish(odom);

}

void initBoostBimap() {
  iclValidMarker.insert(bm_type::value_type(0, 475));
  iclValidMarker.insert(bm_type::value_type(1, 517));
  iclValidMarker.insert(bm_type::value_type(2, 727));
  iclValidMarker.insert(bm_type::value_type(3, 734));
  iclValidMarker.insert(bm_type::value_type(4, 769));
  iclValidMarker.insert(bm_type::value_type(5, 776));
  iclValidMarker.insert(bm_type::value_type(6, 907));
  iclValidMarker.insert(bm_type::value_type(7, 949));
  iclValidMarker.insert(bm_type::value_type(8, 979));
  iclValidMarker.insert(bm_type::value_type(9, 986));
  iclValidMarker.insert(bm_type::value_type(10, 991));
  iclValidMarker.insert(bm_type::value_type(11, 993));
  iclValidMarker.insert(bm_type::value_type(12, 1021));
  iclValidMarker.insert(bm_type::value_type(13, 1028));
  iclValidMarker.insert(bm_type::value_type(14, 1033));
  iclValidMarker.insert(bm_type::value_type(15, 1035));
  iclValidMarker.insert(bm_type::value_type(16, 1159));
  iclValidMarker.insert(bm_type::value_type(17, 1166));
  iclValidMarker.insert(bm_type::value_type(18, 1201));
  iclValidMarker.insert(bm_type::value_type(19, 1208));
  iclValidMarker.insert(bm_type::value_type(20, 1231));
  iclValidMarker.insert(bm_type::value_type(21, 1238));
  iclValidMarker.insert(bm_type::value_type(22, 1243));
  iclValidMarker.insert(bm_type::value_type(23, 1245));
  iclValidMarker.insert(bm_type::value_type(24, 1250));
  iclValidMarker.insert(bm_type::value_type(25, 1252));
  iclValidMarker.insert(bm_type::value_type(26, 1273));
  iclValidMarker.insert(bm_type::value_type(27, 1280));
  iclValidMarker.insert(bm_type::value_type(28, 1285));
  iclValidMarker.insert(bm_type::value_type(29, 1287));
  iclValidMarker.insert(bm_type::value_type(30, 1292));
  iclValidMarker.insert(bm_type::value_type(31, 1294));
}

static void programOptions(ros::NodeHandle &n) {
  n.param<std::string>("topic_in_image", topicInImage, "/genicam/cam4"); // Video parameter for the camera
  n.param<std::string>("image_camera_info", topicCameraInfo, "/genicam/camera_info"); // Camera info for the image
  n.param<std::string>("topic_out_odom", topicOutOdom, "/odom"); // scope for sending the odometries
  n.param<std::string>("topic_out_pixel", topicOutPixel, "/pixel"); // scope for sending the pixel data
  n.param<string>("window_name", windowName, ros::this_node::getName()); // Window Name
  n.param<string>("marker_type", markerType, "bch"); // Available marker types: bch, icl1, art
  n.param<int>("marker_size", markerSize, 100); // Marker Size in mm
  n.param<int>("fps", fps, 15); // FPS
  n.param<string>("fid_config", fidConfigPath, ""); // path to fiducial config
  n.param<int>("focal_length", focalLength, 8); // focal length in millimeter
  n.param<int>("verbose", verbose, 0); // more debug output
  n.param<int>("draw3d_axis", draw3dAxis, 0);
  n.param<int>("mono_color", monocolor, 1);
  n.param<int>("pub_pixel", pubPixelMode, 0);

  if (markerType != "bch" && markerType != "icl1") {
    ROS_WARN("[%s] Markertype \"%s\" is not a valid markertype. Valid markertypes are: bch, icl1.", ros::this_node::getName().c_str(), markerType.c_str());
    exit(0);
  }
  if (markerType == "icl1") {
    initBoostBimap();
    objectnum = iclValidMarker.size();
    std::string result;
    for (auto p : iclValidMarker.left) {
      result = result + std::to_string(p.second) + " ";
    }
    ROS_INFO("[%s] Valid icl1 markerids are: %s.", ros::this_node::getName().c_str(), result.c_str());
  }
}

void callbackCameraInfo(sensor_msgs::CameraInfo msg) {
  if (cameraInfo.K != msg.K) { // if something changed
    cameraInfo = msg;
    setCameraInfo = true;
    ROS_INFO("[%s] Update camera_info.", ros::this_node::getName().c_str());
  }
}

void callbackImage(sensor_msgs::ImageConstPtr msg) {
  if (!setCameraInfo) {
    ROS_WARN("[%s] Tracking abort, no cameraInfo recieved yet.", ros::this_node::getName().c_str());
    return;
  }
  if (msg->encoding == "mono8" || msg->encoding == "rgb8") {
    image = cv_bridge::toCvShare(msg, msg->encoding)->image;
    imageHeader = msg->header;
  } else {
    ROS_WARN("[%s] Unknown Image encoding %s.", ros::this_node::getName().c_str(), msg->encoding.c_str());
    return;
  }

  mainLoopTracking();
}

void initIclParameter() {

  icl::math::FixedMatrix<float, 4, 3> projectionMatrix;
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 4; i++) {
      projectionMatrix.at(i, j) = cameraInfo.P[i + j * 4];
    }
  }
  if (verbose)
    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] " << "projection matrix: " << endl << projectionMatrix);
  icl::geom::Camera cam = icl::geom::Camera::createFromProjectionMatrix(projectionMatrix, focalLength);
  if (verbose)
    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] " << "cam: " << endl << cam);
  if (markerType == "icl1")
    fid = new icl::markers::FiducialDetector(markerType, "[450-1300]", "size=" + std::to_string(markerSize) + "x" + std::to_string(markerSize));
  else
    fid = new icl::markers::FiducialDetector(markerType, objectnumString, "size=" + std::to_string(markerSize) + "x" + std::to_string(markerSize));
//  fid->loadProperties(fidConfigPath);
//  fid->setPropertyValue("pose.algorithm", "SamplingFine");
  fid->setConfigurableID("fid");

  if (verbose) {
    for (string s : fid->getPropertyList()) {
      ROS_INFO_STREAM("[" << ros::this_node::getName() << "] " << "fid config: " << s << ":" << fid->getPropertyValue(s));
    }
  }

  icl::utils::Size s(cameraInfo.width, cameraInfo.height);
  scene.addCamera(cam);
  scene.getCamera(0).setResolution(s);
  fid->setCamera(scene.getCamera(0));


  iclGui << Draw().handle("draw")
         << Prop("fid").maxSize(18, 99)
         << Show();

  static DrawHandle draw_t = iclGui["draw"];
  draw = draw_t;
}

void run() {
  initIclParameter();
  ros::Rate rate(fps);
  while (ros::ok()) {
    ros::spinOnce();
    draw.render();
    rate.sleep();
  }
  ros::shutdown();
  QCoreApplication::exit();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle n("~");
  programOptions(n);

  cameraInfoSub = n.subscribe(topicCameraInfo, 1, callbackCameraInfo);

  // Allocate the publisher for the maximum number of markers
  pubOdom.resize(objectnum);
  pubPixel.resize(objectnum);
  int idx = 0;
  for (auto it = pubOdom.begin(); it != pubOdom.end(); ++it, ++idx) {
    std::stringstream ss;
    int id = idx;
    if (markerType == "icl1")
      id = iclValidMarker.left.at(idx);

    ss << n.getNamespace() << std::string("/") << topicOutOdom << std::string("/") << id;

    *it = n.advertise<nav_msgs::Odometry>(ss.str(), 1);
  }
  idx = 0;
  for (auto it = pubPixel.begin(); it != pubPixel.end(); ++it, ++idx) {
    std::stringstream ss;
    int id = idx;
    if (markerType == "icl1")
      id = iclValidMarker.left.at(idx);

    ss << n.getNamespace() << std::string("/") << topicOutPixel << std::string("/") << id;

    *it = n.advertise<nav_msgs::Odometry>(ss.str(), 1);
  }

  ros::Rate rate(fps);
  while (!setCameraInfo) {
    rate.sleep();
    ros::spinOnce();
  }

  imageSub = n.subscribe(topicInImage, 1, callbackImage);

  return ICLApp(argc, argv, "", initIclParameter, run).exec();
}
