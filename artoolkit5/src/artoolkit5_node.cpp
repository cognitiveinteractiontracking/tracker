/*
 *  Forked from check_id.c in ARToolKit5
 *
 *  Allows visual verification of ARToolKit pattern ID's, including
 *  failure modes, ID numbers, and poses.
 *
 *  Press '?' while running for help on available key commands.
 *
 *  This file is part of ARToolKit.
 *
 *  ARToolKit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ARToolKit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with ARToolKit.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2015 Daqri, LLC.
 *  Copyright 2010-2015 ARToolworks, Inc.
 *
 *  Author(s): Philip Lamb.
 *             Extended by Timo Korthals
 *
 */


// ============================================================================
//    Includes
// ============================================================================

#include <stdio.h>
#include <chrono>
#include <mutex>
#include <atomic>
#include <string.h>

#ifdef _WIN32
#  define snprintf _snprintf
#endif
#include <stdlib.h>                    // malloc(), free()
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
extern "C" {
//#include <AR/videoLuma.h>
}

#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>            // arParamDisp()
#include <AR/ar.h>
#include <AR/gsub_lite.h>
#include <AR/arMulti.h>

// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// openCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// boost
#include <boost/algorithm/string.hpp>

// ============================================================================
//    Constants
// ============================================================================

#define VIEW_DISTANCE_MIN        1.0            // Objects closer to the camera than this will not be displayed.
#define VIEW_DISTANCE_MAX        10000.0        // Objects further away from the camera than this will not be displayed.

typedef struct _cutoffPhaseColours {
  int cutoffPhase;
  GLubyte colour[3];
} cutoffPhaseColours_t ;

const cutoffPhaseColours_t cutoffPhaseColours[AR_MARKER_INFO_CUTOFF_PHASE_DESCRIPTION_COUNT] = {
  {AR_MARKER_INFO_CUTOFF_PHASE_NONE,                               {0xff, 0x0,  0x0 }},  // Red.
  {AR_MARKER_INFO_CUTOFF_PHASE_PATTERN_EXTRACTION,                 {0x95, 0xd6, 0xf6}},  // Light blue.
  {AR_MARKER_INFO_CUTOFF_PHASE_MATCH_GENERIC,                      {0x0,  0x0,  0xff}},  // Blue.
  {AR_MARKER_INFO_CUTOFF_PHASE_MATCH_CONTRAST,                     {0x99, 0x66, 0x33}},  // Brown.
  {AR_MARKER_INFO_CUTOFF_PHASE_MATCH_BARCODE_NOT_FOUND,            {0x7f, 0x0,  0x7f}},  // Purple.
  {AR_MARKER_INFO_CUTOFF_PHASE_MATCH_BARCODE_EDC_FAIL,             {0xff, 0x0,  0xff}},  // Magenta.
  {AR_MARKER_INFO_CUTOFF_PHASE_MATCH_CONFIDENCE,                   {0x0,  0xff, 0x0 }},  // Green.
  {AR_MARKER_INFO_CUTOFF_PHASE_POSE_ERROR,                         {0xff, 0x7f, 0x0 }},  // Orange.
  {AR_MARKER_INFO_CUTOFF_PHASE_POSE_ERROR_MULTI,                   {0xff, 0xff, 0x0 }},  // Yellow.
  {AR_MARKER_INFO_CUTOFF_PHASE_HEURISTIC_TROUBLESOME_MATRIX_CODES, {0xc6, 0xdc, 0x6a}},  // Khaki.
};

// ============================================================================
//    Global variables
// ============================================================================

static int windowed = TRUE;                     // Use windowed (TRUE) or fullscreen mode (FALSE) on launch.
static int windowWidth = 640;                    // Initial window width, also updated during program execution.
static int windowHeight = 480;                  // Initial window height, also updated during program execution.
static int windowDepth = 32;                    // Fullscreen mode bit depth.
static int windowRefresh = 0;                    // Fullscreen mode refresh rate. Set to 0 to use default rate.

static int          gARTImageSavePlease = FALSE;

// Marker detection.
static ARHandle        *gARHandle = NULL;
static ARPattHandle    *gARPattHandle = NULL;
static long            gCallCountMarkerDetect = 0;
static int          gPattSize = AR_PATT_SIZE1;
static int          gPattCountMax = AR_PATT_NUM_MAX;
//static ARVideoLumaInfo* lumaInfo = NULL;
static int _xsize = 0, _ysize = 0; // Window size
static std::vector<std::string> multimarker_patt_names;

// Transformation matrix retrieval.
static AR3DHandle    *gAR3DHandle = NULL;
static int          gRobustFlag = TRUE;
#define CHECK_ID_MULTIMARKERS_MAX 16
static int gMultiConfigCount = 0;
static ARMultiMarkerInfoT *gMultiConfigs[CHECK_ID_MULTIMARKERS_MAX] = {NULL};
static ARdouble gMultiErrs[CHECK_ID_MULTIMARKERS_MAX];

static int monoIsBayer = 0;

// Drawing.
static ARParamLT *gCparamLT = NULL;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
static int gShowHelp = 1;
static int gShowMode = 1;
static GLint gViewport[4];
static int gDrawPatternSize = 0;
static ARUint8 ext_patt[AR_PATT_SIZE2_MAX * AR_PATT_SIZE2_MAX * 3]; // Holds unwarped pattern extracted from image.

// Publish counter of image
int publishImageCounter = 0;
int publishImageRate = 1;  // Unit: [Hz]
bool publishImageBool = false;
int counter = 0;

// History for continues tracking
const int objectnum = 64;  // This is the amount of maximum possible marker IDs
std::vector<bool> markerVisible(objectnum, false);

// Topics for the data
static std::string topicInImage, topicOutOdom, topicOutPixel;
static std::string parentFrameId;

// Gui Options
static int gui = 0;
static std::string windowName("ARToolKit-Tracking");

// Pattern Width in mm
static ARdouble gPatt_width = 80.0;

// AR Parameter
ARdouble pattRatio = (ARdouble)AR_PATT_RATIO;
AR_MATRIX_CODE_TYPE matrixCodeType = AR_MATRIX_CODE_TYPE_DEFAULT;
int labelingMode = AR_DEFAULT_LABELING_MODE;
int patternDetectionMode = AR_DEFAULT_PATTERN_DETECTION_MODE;
AR_LABELING_THRESH_MODE labelingThreshMode = AR_LABELING_THRESH_MODE_AUTO_OTSU;
char *cpara = NULL;
char *vconf = NULL;

static cv::Mat imageRGB_t, imageGRAY_t;

// ============================================================================
//    Function prototypes.
// ============================================================================

// static void usage(char *com);
static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p, ARHandle **arhandle, AR3DHandle **ar3dhandle);
static int setupMarkers(const std::vector<std::string> multimarker_patt_names, ARMultiMarkerInfoT *multiConfigs[], ARHandle *arhandle, ARPattHandle **pattHandle_p, int patternDetectionMode);
static void cleanup(void);
static void Keyboard(unsigned char key, int x, int y);
static void Visibility(int visible);
static void Reshape(int w, int h);
static void Display(void);
static void print(const char *text, const float x, const float y, int calculateXFromRightEdge, int calculateYFromTopEdge);
static void drawBackground(const float width, const float height, const float x, const float y);
static void printHelpKeys();
static void printMode();
static int programOptions(ros::NodeHandle &n);

// ============================================================================
//    Functions
// ============================================================================

std::mutex imageMutex;
static cv::Mat imageColorFromCallback; // Three channel image (don't care if RGB, BGR, or GrayGrayGray)
static cv::Mat imageGrayFromCallback; // Single Luminance channel
std_msgs::Header header;
std::atomic<bool> newDataArrived(false);

void callbackImage(sensor_msgs::ImageConstPtr msg) {

  cv_bridge::CvImageConstPtr cv_ptr;
//   cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    if (cv_ptr->image.channels() == 1) {
        if (monoIsBayer) {
          // Just debayer to gray, because ARToolkit doesn't respect color
          cv::Mat grayImg;
          cv::cvtColor(cv_ptr->image, grayImg, CV_BayerGB2GRAY);
          std::vector<cv::Mat> channels = {grayImg, grayImg, grayImg};
          imageMutex.lock();
          grayImg.copyTo(imageGrayFromCallback);
          cv::merge(channels, imageColorFromCallback);
          imageMutex.unlock();
        } else {
          std::vector<cv::Mat> channels = {cv_ptr->image, cv_ptr->image, cv_ptr->image};
          imageMutex.lock();
          cv_ptr->image.copyTo(imageGrayFromCallback);
          cv::merge(channels, imageColorFromCallback);
          imageMutex.unlock();
        }
    } else if (cv_ptr->image.channels() == 3 ) {
      cv::Mat grayImg;
      cv::cvtColor(cv_ptr->image, grayImg, CV_RGB2GRAY);
      imageMutex.lock();
      grayImg.copyTo(imageGrayFromCallback);
      cv_ptr->image.copyTo(imageColorFromCallback);
      imageMutex.unlock();
    } else {
      ROS_ERROR("unsupported number of channels: %d", cv_ptr->image.channels());
    }
    header = msg->header;
    if (!parentFrameId.empty()) {
      header.frame_id = parentFrameId;
    }
    newDataArrived = true;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

static std::vector<ros::Publisher> pubOdom;
static std::vector<ros::Publisher> pubOdomPixel;
static ros::Subscriber sub;
int main(int argc, char** argv) {
  char glutGamemode[32];

  ros::init(argc, argv, "artoolkit5_node");
  ros::NodeHandle n("~");
  programOptions(n);

  std::string topicBayerExtension = monoIsBayer ? std::string("/bayer") : std::string("");
  sub = n.subscribe(topicInImage + topicBayerExtension, 1, callbackImage);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Allocate the publisher for the maximum number of markers
  pubOdom.resize(objectnum);
  pubOdomPixel.resize(objectnum);
  int idx = 0;
  for (auto it = pubOdom.begin(); it != pubOdom.end(); ++it, ++idx) {
    std::stringstream ss;
    ss << n.getNamespace() << std::string("/") << topicOutOdom << std::string("/") << idx;
    *it = n.advertise<nav_msgs::Odometry>(ss.str(), 1);
  }

  idx = 0;
  for (auto it = pubOdomPixel.begin(); it != pubOdomPixel.end(); ++it, ++idx) {
    std::stringstream ss;
    ss << n.getNamespace() << std::string("/") << topicOutPixel << std::string("/") << idx;
    *it = n.advertise<nav_msgs::Odometry>(ss.str(), 1);
  }

  // Allocate extra publishers for the multimarkers
  for (int i = 0; i <  multimarker_patt_names.size(); ++i) {
    std::stringstream ss;
    ss << n.getNamespace() << std::string("/") << topicOutOdom << std::string("/multimarker/") << i;
    pubOdom.push_back(n.advertise<nav_msgs::Odometry>(ss.str(), 1));
  }

  glutInit(&argc, argv);

  if (!setupCamera(cpara, vconf, &gCparamLT, &gARHandle, &gAR3DHandle)) {
    ROS_INFO("main(): Unable to set up AR camera.\n");
    exit(-1);
  }

  //
  // AR init.
  //
  if (!setupMarkers(multimarker_patt_names, gMultiConfigs, gARHandle, &gARPattHandle, patternDetectionMode)) {
    ROS_ERROR("main(): Unable to set up AR marker(s).");
    cleanup();
    exit(-1);
  }
  gMultiConfigCount = multimarker_patt_names.size();

  arSetLabelingMode(gARHandle, labelingMode);
  arSetPattRatio(gARHandle, pattRatio);
  arSetMatrixCodeType(gARHandle, matrixCodeType);
  arSetLabelingThreshMode(gARHandle, labelingThreshMode);

  //
  // Graphics setup.
  //

  if (gui) {
    // Set up GL context(s) for OpenGL to draw into.
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    if (!windowed) {
      if (windowRefresh) sprintf(glutGamemode, "%ix%i:%i@%i", windowWidth, windowHeight, windowDepth, windowRefresh);
      else sprintf(glutGamemode, "%ix%i:%i", windowWidth, windowHeight, windowDepth);
      glutGameModeString(glutGamemode);
      glutEnterGameMode();
    } else {
      glutInitWindowSize(_xsize/2, _ysize/2);
      glutCreateWindow(argv[0]);
    }
  } else {
    glutInitWindowSize(1, 1);
    glutCreateWindow(argv[0]);
  }
  glutSetWindowTitle(windowName.c_str());
  glutSetIconTitle(windowName.c_str());

  // Setup ARgsub_lite library for current OpenGL context.
  if ((gArglSettings = arglSetupForCurrentContext(&(gCparamLT->param), arVideoGetPixelFormat())) == NULL) {
    ROS_INFO("main(): arglSetupForCurrentContext() returned error.\n");
    cleanup();
    exit(-1);
  }

  arglSetupDebugMode(gArglSettings, gARHandle);
  arUtilTimerReset();

  // Register GLUT event-handling callbacks.
  // NB: mainLoop() is registered by Visibility.
  if (gui) {
    glutDisplayFunc(Display);
    glutReshapeFunc(Reshape);
  }
  glutVisibilityFunc(Visibility);
  glutKeyboardFunc(Keyboard);

  glutMainLoop();

  return (0);
}

static int programOptions(ros::NodeHandle &n) {

  // Program option temp objects
  float boarder_t = 0.0f;
  double pattRatio_t = 0.0;
  double pattWidth_t = 0.0;
  std::string matrixCode_t;
  std::string labelingMode_t;
  std::string patternDetectionMode_t;
  std::string logLevel_t;
  std::string cparam_t;
  std::string labelingTreshMode_t;
  AR_LABELING_THRESH_MODE modea;


  n.param<int>("gui", gui, false); // Show the rectified image by OpenCV
  n.param<std::string>("topic_in_image", topicInImage, "/genicam/cam4"); // Video parameter for the camera
  n.param<std::string>("cpara", cparam_t, "/tmp/file"); // Camera parameter file for the camera
  n.param<std::string>("topic_out_odom", topicOutOdom, "/odom"); // scope for sending the odometries
  n.param<std::string>("topic_out_pixel", topicOutPixel, "/pixel"); // scope for sending the pixel data
  n.param<double>("patt_width",pattWidth_t, 80); // Marker pattern width in mm
  n.param<double>("patt_ratio",pattRatio_t, AR_PATT_RATIO); // Specify the proportion of the marker width/height, occupied by the marker pattern. Range (0.0 - 1.0) (not inclusive) (I.e. 1.0 - 2*borderSize). Default value is 0.5.
  n.param<int>("patt_size", gPattSize, AR_PATT_SIZE1); // Specify the number of rows and columns in the pattern space for template (pictorial) markers. Default value 16 (required for compatibility with ARToolKit prior to version 5.2).
  n.param<int>("patt_count_max", gPattCountMax, AR_PATT_NUM_MAX); // Specify the maximum number of template (pictorial) marker that may be loaded for use in a single matching pass.
  n.param<int>("mono_is_bayer", monoIsBayer, 0); // Treat mono images as bayer and do a debayering
  n.param<float>("border", boarder_t, boarder_t); // specify the width of the pattern border, as a percentage of the marker width. Range (0.0 - 0.5) (not inclusive). (I.e. (1.0 - pattRatio)/2). Default value is 0.25.
  n.param<std::string>("matrix_code_type", matrixCode_t, "AR_MATRIX_CODE_3x3"); // specify the type of matrix code used, choose one: AR_MATRIX_CODE_3x3, AR_MATRIX_CODE_3x3_HAMMING63, AR_MATRIX_CODE_3x3_PARITY65, AR_MATRIX_CODE_4x4, AR_MATRIX_CODE_4x4_BCH_13_9_3, AR_MATRIX_CODE_4x4_BCH_13_5_5,AR_MATRIX_CODE_5x5 AR_MATRIX_CODE_6x6, AR_MATRIX_CODE_GLOBAL_ID
  n.param<std::string>("labeling_mode", labelingMode_t, "AR_LABELING_BLACK_REGION"); // AR_LABELING_BLACK_REGION or AR_LABELING_WHITE_REGION
  n.param<std::string>("pattern_detection_mode", patternDetectionMode_t, "AR_TEMPLATE_MATCHING_COLOR"); // specify the pattern detection mode,choose one: AR_TEMPLATE_MATCHING_COLOR, AR_TEMPLATE_MATCHING_MONO, AR_MATRIX_CODE_DETECTION, AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX, AR_TEMPLATE_MATCHING_MONO_AND_MATRIX
  n.param<std::string>("log_level", logLevel_t, "REL_INFO"); // REL_INFO(NO LOG), DEBUG, INFO, WARN, ERROR
  n.param<std::string>("labeling_thresh_mode", labelingTreshMode_t, "AR_LABELING_THRESH_MODE_AUTO_OTSU"); // specifiy the labeling threshold mode, choose between: AR_LABELING_THRESH_MODE_AUTO_OTSU, AR_LABELING_THRESH_MODE_AUTO_MEDIAN, AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE, AR_LABELING_THRESH_MODE_AUTO_BRACKETING
  n.param<std::string>("window_name", windowName, "Tracking"); // Window name
  n.param<std::string>("parent_frame_id", parentFrameId, ""); // Use parent frame by image if not set
  std::string multimarker_patt_names_csv;
  n.param<std::string>("multimarker_patt_names", multimarker_patt_names_csv, ""); // Location of multimarker files as comma-seperated list

  // Load the csv for multimarker files
  if (!multimarker_patt_names_csv.empty()) {
    boost::split(multimarker_patt_names, multimarker_patt_names_csv, [](char c){return c == ',';});
    ROS_INFO("Multimarker patterns defined:");
    if ( multimarker_patt_names.size() > CHECK_ID_MULTIMARKERS_MAX) {
      multimarker_patt_names.resize(CHECK_ID_MULTIMARKERS_MAX);
      ROS_WARN("Number of multimarkers limited to %d", CHECK_ID_MULTIMARKERS_MAX);
    }
    for (int i = 0; i < multimarker_patt_names.size(); ++i) {
      ROS_INFO_STREAM(i << ":" << multimarker_patt_names.at(i));
    }
  }

  cpara = strdup(cparam_t.c_str());

  if (pattRatio_t != 0.0) {
    pattRatio = (ARdouble)pattRatio_t;
  }

  if (boarder_t != 0.0f) {
    pattRatio = (ARdouble)(1.0f - 2.0f * boarder_t);
  }
  gPatt_width = (ARdouble)pattWidth_t;

  if (matrixCode_t == "AR_MATRIX_CODE_3x3") matrixCodeType = AR_MATRIX_CODE_3x3;
  else if (matrixCode_t == "AR_MATRIX_CODE_3x3_HAMMING63") matrixCodeType = AR_MATRIX_CODE_3x3_HAMMING63;
  else if (matrixCode_t == "AR_MATRIX_CODE_3x3_PARITY65") matrixCodeType = AR_MATRIX_CODE_3x3_PARITY65;
  else if (matrixCode_t == "AR_MATRIX_CODE_4x4") matrixCodeType = AR_MATRIX_CODE_4x4;
  else if (matrixCode_t == "AR_MATRIX_CODE_4x4_BCH_13_9_3") matrixCodeType = AR_MATRIX_CODE_4x4_BCH_13_9_3;
  else if (matrixCode_t == "AR_MATRIX_CODE_4x4_BCH_13_5_5") matrixCodeType = AR_MATRIX_CODE_4x4_BCH_13_5_5;
  else if (matrixCode_t == "AR_MATRIX_CODE_5x5") matrixCodeType = AR_MATRIX_CODE_5x5;
  else if (matrixCode_t == "AR_MATRIX_CODE_6x6") matrixCodeType = AR_MATRIX_CODE_6x6;
  else if (matrixCode_t == "AR_MATRIX_CODE_GLOBAL_ID") matrixCodeType = AR_MATRIX_CODE_GLOBAL_ID;
  else  {
    std::cout << "Error: argument '" << matrixCode_t << "' to --matrixCodeType invalid." << std::endl;
    return -1;
  }

  if (labelingMode_t == "AR_LABELING_BLACK_REGION") labelingMode = AR_LABELING_BLACK_REGION;
  else if (labelingMode_t == "AR_LABELING_WHITE_REGION") labelingMode = AR_LABELING_WHITE_REGION;
  else {
    std::cout << "Error: argument '" << labelingMode_t << "' to --labelingMode invalid." << std::endl;
    return -1;
  }
  if (patternDetectionMode_t == "AR_TEMPLATE_MATCHING_COLOR") patternDetectionMode = AR_TEMPLATE_MATCHING_COLOR;
  else if (patternDetectionMode_t == "AR_TEMPLATE_MATCHING_MONO") patternDetectionMode = AR_TEMPLATE_MATCHING_MONO;
  else if (patternDetectionMode_t == "AR_MATRIX_CODE_DETECTION") patternDetectionMode = AR_MATRIX_CODE_DETECTION;
  else if (patternDetectionMode_t == "AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX") patternDetectionMode = AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX;
  else if (patternDetectionMode_t == "AR_TEMPLATE_MATCHING_MONO_AND_MATRIX") patternDetectionMode = AR_TEMPLATE_MATCHING_MONO_AND_MATRIX;
  else {
    std::cout <<  "Error: argument '" << patternDetectionMode_t << "' to --patternDetectionMode invalid." << std::endl;
    return -1;
  }
  if (logLevel_t == "DEBUG") arLogLevel = AR_LOG_LEVEL_DEBUG;
  else if (logLevel_t == "INFO") arLogLevel = AR_LOG_LEVEL_INFO;
  else if (logLevel_t == "WARN") arLogLevel = AR_LOG_LEVEL_WARN;
  else if (logLevel_t == "ERROR") arLogLevel = AR_LOG_LEVEL_ERROR;
  else if (logLevel_t == "REL_INFO") arLogLevel = AR_LOG_LEVEL_REL_INFO;
  else {
    std::cout <<  "Error: argument '" << logLevel_t << "' to --loglevel invalid." << std::endl;
    return -1;
  }

  if (labelingTreshMode_t == "AR_LABELING_THRESH_MODE_AUTO_OTSU") labelingThreshMode = AR_LABELING_THRESH_MODE_AUTO_OTSU;
  else if (labelingTreshMode_t == "AR_LABELING_THRESH_MODE_AUTO_MEDIAN") labelingThreshMode = AR_LABELING_THRESH_MODE_AUTO_MEDIAN;
  else if (labelingTreshMode_t == "AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE") labelingThreshMode = AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE;
  else if (labelingTreshMode_t == "AR_LABELING_THRESH_MODE_AUTO_BRACKETING") labelingThreshMode = AR_LABELING_THRESH_MODE_AUTO_BRACKETING;
  else {
    std::cout <<  "Error: argument '" << labelingTreshMode_t << "' to --labelingThreshMode invalid." << std::endl;
    return -1;
  }
  return 0;
}

static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p, ARHandle **arhandle, AR3DHandle **ar3dhandle) {
  ARParam            cparam;
  int                xsize = 0, ysize = 0;

// Load the camera parameters, resize for the window and init.
  if (arParamLoad(cparam_name, 1, &cparam) < 0) {
    ROS_INFO("setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
    arVideoClose();
    return (FALSE);
  }

  _xsize = cparam.xsize;
  _ysize = cparam.ysize;

  // Open the video path.
  // pseudo fakessrc to initialize glib params.
  std::string conf = "videotestsrc ! video/x-raw-rgb,bpp=24,width=" + std::to_string(_xsize) + ",height=" + std::to_string(_ysize) +" ! identity name=artoolkit sync=true ! fakesink";
  if (arVideoOpen(conf.c_str()) < 0)  {
    ROS_INFO("setupCamera(): Unable to open connection to camera.\n");
    return (FALSE);
  }

  ARLOG("*** Camera Parameter ***\n");
  arParamDisp(&cparam);

  if ((*cparamLT_p = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
    ROS_INFO("setupCamera(): Error: arParamLTCreate.\n");
    return (FALSE);
  }
  if ((*arhandle = arCreateHandle(*cparamLT_p)) == NULL) {
    ROS_INFO("setupCamera(): Error: arCreateHandle.\n");
    return (FALSE);
  }
// if (arSetPixelFormat(*arhandle, pixFormat) < 0) {
//   ROS_INFO("setupCamera(): Error: arSetPixelFormat.\n");
//   return (FALSE);
// }
  if (arSetDebugMode(*arhandle, AR_DEBUG_DISABLE) < 0) {
    ROS_INFO("setupCamera(): Error: arSetDebugMode.\n");
    return (FALSE);
  }
  if ((*ar3dhandle = ar3DCreateHandle(&cparam)) == NULL) {
    ROS_INFO("setupCamera(): Error: ar3DCreateHandle.\n");
    return (FALSE);
  }

// Allocate space for the luminance image
//  lumaInfo = arVideoLumaInit(_xsize, _ysize, arVideoGetPixelFormat());
  return (TRUE);
}

///
/// Setup the markers
/// param multi_patt_count Number of multimarker patterns to load (if 0, standard pattern mode is chosen)
/// 

static int setupMarkers(std::vector<std::string> multimarker_patt_names, ARMultiMarkerInfoT * multiConfigs[], ARHandle * arhandle, ARPattHandle **pattHandle_p, int patternDetectionMode) {
  int i;

  if (multimarker_patt_names.size() == 0) {
    // Default behaviour is to default to matrix mode.
    *pattHandle_p = NULL;
    arSetPatternDetectionMode( arhandle, patternDetectionMode ); // If no markers specified, default to matrix mode.
  } else {
    // If marker configs have been specified, attempt to load them.

    int mode = -1, nextMode;

    // Need a pattern handle because the config file could specify matrix or template markers.
    if ((*pattHandle_p = arPattCreateHandle2(gPattSize, gPattCountMax)) == NULL) {
      ROS_INFO("setupMarkers(): Error: arPattCreateHandle2.\n");
      return (FALSE);
    }

    for (i = 0; i < multimarker_patt_names.size(); i++) {

      if (!(multiConfigs[i] = arMultiReadConfigFile(multimarker_patt_names.at(i).c_str(), *pattHandle_p))) {
        ROS_INFO("setupMarkers(): Error reading multimarker config file '%s'.\n", multimarker_patt_names.at(i).c_str());
        for (i--; i >= 0; i--) {
          arMultiFreeConfig(multiConfigs[i]);
        }
        arPattDeleteHandle(*pattHandle_p);
        return (FALSE);
      }

      // HACK min_submarker is uninitialized by artoolkit 5.3.3
      multiConfigs[i]->min_submarker = 0;

      if (multiConfigs[i]->patt_type == AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE) {
        nextMode = AR_TEMPLATE_MATCHING_COLOR;
      } else if (multiConfigs[i]->patt_type == AR_MULTI_PATTERN_DETECTION_MODE_MATRIX) {
        nextMode = AR_MATRIX_CODE_DETECTION;
      } else { // AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE_AND_MATRIX or mixed.
        nextMode = AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX;
      }

      if (mode == -1) {
        mode = nextMode;
      } else if (mode != nextMode) {
        mode = AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX;
      }
    }
    arSetPatternDetectionMode(arhandle, mode);

    arPattAttach(arhandle, *pattHandle_p);
  }


  return (TRUE);
}

static void cleanup(void) {
  int i;

  arglCleanup(gArglSettings);
  gArglSettings = NULL;

  arPattDetach(gARHandle);
  for (i = 0; i < gMultiConfigCount; i++) {
    arMultiFreeConfig(gMultiConfigs[i]);
  }
  if (gARPattHandle) arPattDeleteHandle(gARPattHandle);

//  arVideoLumaFinal(&lumaInfo);
  arDeleteHandle(gARHandle);
  arParamLTFree(&gCparamLT);
  arVideoClose();
}

static void Keyboard(unsigned char key, int x, int y) {
  int mode, threshChange = 0;
  AR_LABELING_THRESH_MODE modea;

  switch (key) {
  case 0x1B:                        // Quit.
  case 'Q':
  case 'q':
    cleanup();
    exit(0);
    break;
  case 'X':
  case 'x':
    arGetImageProcMode(gARHandle, &mode);
    switch (mode) {
    case AR_IMAGE_PROC_FRAME_IMAGE:  mode = AR_IMAGE_PROC_FIELD_IMAGE; break;
    case AR_IMAGE_PROC_FIELD_IMAGE:
    default: mode = AR_IMAGE_PROC_FRAME_IMAGE; break;
    }
    arSetImageProcMode(gARHandle, mode);
    break;
  case 'C':
  case 'c':
    ROS_INFO("*** Camera - %f (frame/sec)\n", (double)gCallCountMarkerDetect / arUtilTimer());
    gCallCountMarkerDetect = 0;
    arUtilTimerReset();
    break;
  case 'a':
  case 'A':
    arGetLabelingThreshMode(gARHandle, &modea);
    switch (modea) {
    case AR_LABELING_THRESH_MODE_MANUAL:        modea = AR_LABELING_THRESH_MODE_AUTO_MEDIAN; break;
    case AR_LABELING_THRESH_MODE_AUTO_MEDIAN:   modea = AR_LABELING_THRESH_MODE_AUTO_OTSU; break;
    case AR_LABELING_THRESH_MODE_AUTO_OTSU:     modea = AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE; break;
    case AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE: modea = AR_LABELING_THRESH_MODE_AUTO_BRACKETING; break;
    case AR_LABELING_THRESH_MODE_AUTO_BRACKETING:
    default: modea = AR_LABELING_THRESH_MODE_MANUAL; break;
    }
    arSetLabelingThreshMode(gARHandle, modea);
    break;
  case '-':
    threshChange = -5;
    break;
  case '+':
  case '=':
    threshChange = +5;
    break;
  case 'D':
  case 'd':
    arGetDebugMode(gARHandle, &mode);
    arSetDebugMode(gARHandle, !mode);
    break;
  case 's':
  case 'S':
    if (!gARTImageSavePlease) gARTImageSavePlease = TRUE;
    break;
  case '?':
  case '/':
    gShowHelp++;
    if (gShowHelp > 2) gShowHelp = 0;
    break;
  case 'r':
  case 'R':
    gRobustFlag = !gRobustFlag;
    break;
  case 'm':
  case 'M':
    gShowMode = !gShowMode;
    break;
  default:
    break;
  }
  if (threshChange) {
    int threshhold;
    arGetLabelingThresh(gARHandle, &threshhold);
    threshhold += threshChange;
    if (threshhold < 0) threshhold = 0;
    if (threshhold > 255) threshhold = 255;
    arSetLabelingThresh(gARHandle, threshhold);
  }

}

static void mainLoop(void) {
  int i;
  static int imageNumber = 0;
  static int ms_prev;
  int ms;
  float s_elapsed;
  AR2VideoBufferT imageCv;
  int pattDetectMode;
  AR_MATRIX_CODE_TYPE matrixCodeType;

  // Grab a video frame.


  static std_msgs::Header header_t;
  if (newDataArrived) {
      imageMutex.lock();
      imageColorFromCallback.copyTo(imageRGB_t);
      imageGrayFromCallback.copyTo(imageGRAY_t);
      header_t = header;
      newDataArrived = false;
      imageMutex.unlock();
  } else {
      ROS_DEBUG("Skipping processing because now new image has arrived yet");
      return;
  }
  if(imageRGB_t.rows == 0 || imageRGB_t.cols == 0) {
      ROS_WARN_STREAM_ONCE("No content to visualize (x,y): " << imageRGB_t.rows << " " << imageRGB_t.cols);
      return;
  }

  std::chrono::microseconds time = std::chrono::duration_cast< std::chrono::microseconds >(
      std::chrono::system_clock::now().time_since_epoch());
  imageCv.buff = imageRGB_t.data;
  imageCv.buffLuma = imageGRAY_t.data;
  imageCv.bufPlaneCount = 0;
  imageCv.bufPlanes = NULL;
  imageCv.fillFlag = 1;
  imageCv.buffLuma = imageGRAY_t.data;
  imageCv.time_sec = ARUint32(time.count() / 1e6);
  imageCv.time_usec = ARUint32(time.count() % uint64_t(1e6));

  // Start the tracking
//   if (arLogLevel != AR_LOG_LEVEL_REL_INFO) {
//     system("clear");
//   }
  if (gui) {
    arglPixelBufferDataUpload(gArglSettings, imageCv.buff);
  }

  if (gARTImageSavePlease) {
    char imageNumberText[15];
    sprintf(imageNumberText, "image-%04d.jpg", imageNumber++);
    if (arVideoSaveImageJPEG(gARHandle->xsize, gARHandle->ysize, gARHandle->arPixelFormat, imageCv.buff, imageNumberText, 75, 0) < 0) {
      ROS_INFO("Error saving video image.\n");
    }
    gARTImageSavePlease = FALSE;
  }

  gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.
  // Detect the markers in the video frame.
  if (arDetectMarker(gARHandle, &imageCv) < 0) {
    ROS_ERROR("arDetectMarker failed");
    exit(-1);
  }

  // If  marker config files were specified, evaluate detected patterns against them now.
  for (i = 0; i < gMultiConfigCount; i++) {
    if (gRobustFlag) gMultiErrs[i] = arGetTransMatMultiSquareRobust(gAR3DHandle, arGetMarker(gARHandle), arGetMarkerNum(gARHandle), gMultiConfigs[i]);
    else gMultiErrs[i] = arGetTransMatMultiSquare(gAR3DHandle, arGetMarker(gARHandle), arGetMarkerNum(gARHandle), gMultiConfigs[i]);
    if (gMultiConfigs[i]->prevF != 0) ROS_INFO("Found multimarker set %d, err=%0.3f\n", i, gMultiErrs[i]);
  }

  // For matrix mode, draw the pattern image of the largest marker.
  arGetPatternDetectionMode(gARHandle, &pattDetectMode);
  arGetMatrixCodeType(gARHandle, &matrixCodeType);
  if (pattDetectMode == AR_MATRIX_CODE_DETECTION || pattDetectMode == AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX || pattDetectMode == AR_TEMPLATE_MATCHING_MONO_AND_MATRIX ) {

    int area = 0, biggestMarker = -1;


    // Take only the marker with the highest confidence, if marker has same ID
    int j, k;
    for (j = 0; j < gARHandle->marker_num; j++) {
      for (k = 0; k < gARHandle->marker_num; k++) {
        if (gARHandle->markerInfo[j].id == gARHandle->markerInfo[k].id && gARHandle->markerInfo[j].id >= 0) {
          if (gARHandle->markerInfo[j].cf > gARHandle->markerInfo[k].cf) {
            gARHandle->markerInfo[k].id = -1;
          }
        }
      }
    }

    // Print some information
    ROS_INFO("---- Start marker output (Only valid markers are printed)");
    char buffer[100];
    // Single marker tracking
    std::stringstream ss_MarkerInfo;
    ss_MarkerInfo << "Marker detections:\n";
    for (j = 0; j < gARHandle->marker_num; j++) {
      if (gARHandle->markerInfo[j].id == -1) continue;
      ss_MarkerInfo <<"Marker Info ID: " << j << ":\n" <<
                      "\tarea     : "         << gARHandle->markerInfo[j].area     << "\n" <<
                      "\tid       : "         << gARHandle->markerInfo[j].id       << "\n" <<
                      "\tidPatt   : "         << gARHandle->markerInfo[j].idPatt   << "\n" <<
                      "\tidMatrix : "         << gARHandle->markerInfo[j].idMatrix << "\n" <<
                      "\tdir      : "         << gARHandle->markerInfo[j].dir      << "\n" <<
                      "\tdirPatt  : "         << gARHandle->markerInfo[j].dirPatt  << "\n" <<
                      "\tdirMatrix: "         << gARHandle->markerInfo[j].dirMatrix<< "\n" <<
                      "\tcf       : "         << gARHandle->markerInfo[j].cf       << "\n" <<
                      "\tcfPatt   : "         << gARHandle->markerInfo[j].cfPatt   << "\n" <<
                      "\tcfMatrix : "         << gARHandle->markerInfo[j].cfMatrix << "\n" <<
                      "\tpos(x,y) : (" << gARHandle->markerInfo[j].pos[0] << ", " << gARHandle->markerInfo[j].pos[1] << "\n" <<
                      "\terrorCorrected: "    << gARHandle->markerInfo[j].errorCorrected << "\n" <<
                      "\tglobalID:       "    << (int)gARHandle->markerInfo[j].globalID;


      ss_MarkerInfo << "\nVerticies:\n";
      int vid, upperLeftCornerId;
      for (vid = 0; vid < 4; ++vid) {
        if ((4 - gARHandle->markerInfo[j].dir) % 4 == vid) { ss_MarkerInfo << "x "; upperLeftCornerId = vid;} // is upper left corner
        else {ss_MarkerInfo << "  ";}
        ss_MarkerInfo << " Vertex" << vid << "(x,y) : (" << gARHandle->markerInfo[j].vertex[vid][0] << ", " << gARHandle->markerInfo[j].vertex[vid][1] << ")\n";
      }
      ss_MarkerInfo << "\n";

      // Naive orientation in grad (Take the angular between the upper left corner and the center of the marker)
      double orientation = atan2(gARHandle->markerInfo[j].vertex[upperLeftCornerId][1] - gARHandle->markerInfo[j].pos[1],
                                  gARHandle->markerInfo[j].vertex[upperLeftCornerId][0] - gARHandle->markerInfo[j].pos[0]) *
                            180 / M_PI;

      sprintf(buffer, "fz_naive: %.2f °\n", orientation - 45 /* Because we sit in a corner*/ + 180 /*backwards compatibility*/);
      ss_MarkerInfo << buffer;

      static ARdouble   gPatt_trans[3][4];    // Per-marker transformation matrix
      // TODO Replace with arGetTransMatSquareCont from (https://artoolkit.org/documentation/doku.php?id=8_Advanced_Topics:about_faq)

      int err;
      if (!markerVisible.at(gARHandle->markerInfo[j].id))
        err = arGetTransMatSquare(gAR3DHandle, &(gARHandle->markerInfo[j]), gPatt_width, gPatt_trans);
      else
        err = arGetTransMatSquareCont(gAR3DHandle, &(gARHandle->markerInfo[j]), gPatt_trans, gPatt_width, gPatt_trans);

      if(gPatt_trans[2][3] > 5000 || gPatt_trans[2][3] < 0) { //hack
        markerVisible.at(gARHandle->markerInfo[j].id) = false;
        ARLOGe("False calculation of transformation-matrix: id: %d", gARHandle->markerInfo[j].id);
        continue;
      }

      // Set the information, which marker were visible in this process
      for (i = 0; i < objectnum; i++) {
        if (i == gARHandle->markerInfo[j].id) {
          markerVisible.at(i) = true;
        } else
          markerVisible.at(i) = false;
      }

      ss_MarkerInfo << "\nTransformation-Matrix:\n";
      int idx, idy;
      for (idy = 0; idy < 3; ++idy) {
        for (idx = 0; idx < 4; ++idx) {
          if (gPatt_trans[idy][idx] >= 0.0) ss_MarkerInfo << " ";
          sprintf(buffer, "%.2f\t", gPatt_trans[idy][idx]);
          ss_MarkerInfo << buffer;
        }
        ss_MarkerInfo << "\n";
      }
      ss_MarkerInfo << "\n";

      double rot[3][3];
      for (idy = 0; idy < 3; idy++) {
        for (idx = 0; idx < 3; idx++) {
          rot[idy][idx] = gPatt_trans[idy][idx];
        }
      }
      double px_m = 7.4 * 1e-6;
      auto x_px = gPatt_trans[0][3];
      auto y_px = gPatt_trans[1][3];
      auto s_m  = gPatt_trans[2][3] * 1e-3;
      double w_px = sqrt(x_px * x_px + y_px * y_px);
      double w_m = w_px * px_m;
      double f_m = 2.8 * 1e-3; // Focus in meter
      double gamma_rad = atan2(f_m, w_px);
      double r_m = s_m * sin(gamma_rad);
      double xi = w_px / r_m; // camera plane to real life ratio (projection)
      double x_m = x_px / xi;
      double y_m = y_px / xi;
      double z_m = sqrt(s_m * s_m - r_m);

//        ROS_INFO("----------------\n");
//        ROS_INFO("w_px: %.10f\n", w_px);
//        ROS_INFO("w_m: %.10f\n", w_m);
//        ROS_INFO("f_m: %.10f\n", f_m);
//        ROS_INFO("gamma_rad: %.10f\n", gamma_rad);
//        ROS_INFO("r_m: %.10f\n", r_m);
//        ROS_INFO("xi: %.10f\n", xi);
//        ROS_INFO("x_m: %.10f\n", x_m);
//        ROS_INFO("y_m: %.10f\n", y_m);
//        ROS_INFO("z_m: %.10f\n\n", z_m);

      double arQuat[4], arPos[3];
      arUtilMat2QuatPos (gPatt_trans, arQuat, arPos);
      double quat[4], pos[3];
      const double AR_TO_ROS = 0.001;

      pos[0] = arPos[0] * AR_TO_ROS;
      pos[1] = arPos[1] * AR_TO_ROS;
      pos[2] = arPos[2] * AR_TO_ROS;
      //pos[2] = sqrt(pos[2]*pos[2] - sqrt(pos[1]*pos[1] + pos[0]*pos[0]));

      quat[0] = -arQuat[0];
      quat[1] = -arQuat[1];
      quat[2] = -arQuat[2];
      quat[3] = arQuat[3];
      sprintf (buffer, "Pos x: %3.5f  y: %3.5f  z: %3.5f\n", pos[0], pos[1], pos[2]);
      ss_MarkerInfo << buffer;
      sprintf (buffer, "Quat qx: %3.5f qy: %3.5f qz: %3.5f qw: %3.5f\n", quat[0], quat[1], quat[2], quat[3]);
      ss_MarkerInfo << buffer;


      // Quat to Euler
      double ysqr = quat[1] * quat[1];
      double t0 = -2.0f * (ysqr + quat[2] * quat[2]) + 1.0f;
      double t1 = +2.0f * (quat[0] * quat[1] - quat[3] * quat[2]);
      double t2 = -2.0f * (quat[0] * quat[2] + quat[3] * quat[1]);
      double t3 = +2.0f * (quat[1] * quat[2] - quat[3] * quat[0]);
      double t4 = -2.0f * (quat[0] * quat[0] + ysqr) + 1.0f;
      t2 = t2 > 1.0f ? 1.0f : t2;
      t2 = t2 < -1.0f ? -1.0f : t2;
      double rotx = asin(t2);  //pitch
      double roty = atan2(t3, t4); //roll
      double rotz = atan2(t1, t0); //yaw
      rotx = fmod(rotx / M_PI * 180 + 360, 360.0);
      roty = fmod(roty / M_PI * 180 + 360, 360.0);
      rotz = fmod(rotz / M_PI * 180 + 360, 360.0);
      // Old calculation
      // double rotx = atan2(rot[2][1], rot[2][2]);
      // double roty = atan2(-rot[2][0], sqrt(pow(rot[0][0], 2) + pow(rot[1][0], 2)));
      // double rotz = atan2(rot[1][0], rot[0][0]);
      // //calc: radial to grad
      // rotx = fmod(rotx / M_PI * 180 + 360, 360.0);
      // roty = fmod(roty / M_PI * 180 + 360, 360.0);
      // rotz = fmod(rotz / M_PI * 180 + 360, 360.0);
      sprintf (buffer, "fy: %.2f °", roty);
      ss_MarkerInfo << buffer;
      sprintf (buffer, "fx: %.2f °", rotx);
      ss_MarkerInfo << buffer;
      sprintf (buffer, "fz: %.2f °", rotz);
      ss_MarkerInfo << buffer;

      // HACK We try out if this orientation works better
      rotx = 0.0;
      roty = 0.0;
      rotz = orientation;


      // SEND THE VALID MARKER
      if (gARHandle->markerInfo[j].id < 0)
        continue;

      if (gARHandle->markerInfo[j].idPatt < objectnum) {
          nav_msgs::Odometry odom, pixel;
          odom.header = header_t;
          odom.child_frame_id = std::string("base_link/") + std::to_string(gARHandle->markerInfo[j].id);
          odom.pose.pose.position.x = pos[0];
          odom.pose.pose.position.y = pos[1];
          odom.pose.pose.position.z = pos[2];
          odom.pose.pose.orientation.x = quat[0];
          odom.pose.pose.orientation.y = quat[1];
          odom.pose.pose.orientation.z = quat[2];
          odom.pose.pose.orientation.w = quat[3];
          const boost::array<double, 36> covariance = {{
                .1, 0, 0, 0, 0, 0,
                0, .1, 0, 0, 0, 0,
                0, 0, .15, 0, 0, 0,
                0, 0, 0, .01, 0, 0,
                0, 0, 0, 0, .01, 0,
                0, 0, 0, 0, 0, .01}};
          odom.pose.covariance = covariance;
        pixel = odom;
        pixel.pose.pose.position.x = gARHandle->markerInfo[j].pos[0];
        pixel.pose.pose.position.y = gARHandle->markerInfo[j].pos[1];
        pixel.pose.pose.position.z = 0;
        pubOdom.at(gARHandle->markerInfo[j].id).publish(odom);
        pubOdomPixel.at(gARHandle->markerInfo[j].id).publish(pixel);
      }
      // i < 4: a marker got 4 vertices
//        for (int i = 0; i < 4; i++) {
//        twbTracking::proto::Translation *trans = poly->add_vertex();
//          trans->set_x(static_cast<float>(
//                         gARHandle->markerInfo[j].vertex[i][0]));
//          trans->set_y(static_cast<float>(
//                         gARHandle->markerInfo[j].vertex[i][1]));
//        }
      ROS_INFO_STREAM(ss_MarkerInfo.str());
    }

    // Multi marker tracking
    std::stringstream ss_multiMarker;
    ss_multiMarker << "Multi marker detections: ";
    if (gMultiConfigCount > 0) {
      for (k = 0; k < gMultiConfigCount; k++) {
        if (gMultiConfigs[k]->prevF != 0) {
          ss_multiMarker << "\n";
          double arQuat[4], arPos[3];
          arUtilMat2QuatPos (gMultiConfigs[k]->trans, arQuat, arPos);
          double quat[4], pos[3];
          const double AR_TO_ROS = 0.001;

          pos[0] = arPos[0] * AR_TO_ROS;
          pos[1] = arPos[1] * AR_TO_ROS;
          pos[2] = arPos[2] * AR_TO_ROS;
          //pos[2] = sqrt(pos[2]*pos[2] - sqrt(pos[1]*pos[1] + pos[0]*pos[0]));

          quat[0] = -arQuat[0];
          quat[1] = -arQuat[1];
          quat[2] = -arQuat[2];
          quat[3] = arQuat[3];

          sprintf(buffer, "%d:\nPos x: %3.5f  y: %3.5f  z: %3.5f\n",k, pos[0], pos[1], pos[2]);
          ss_multiMarker << buffer;
          sprintf(buffer, "Quat qx: %3.5f qy: %3.5f qz: %3.5f qw: %3.5f\n", quat[0], quat[1], quat[2], quat[3]);
          ss_multiMarker << buffer;

          // Publish the marker
          nav_msgs::Odometry odom;
          odom.header = header_t;
          odom.child_frame_id = std::string("base_link/multimarker/") + std::to_string(k);
          odom.pose.pose.position.x = pos[0];
          odom.pose.pose.position.y = pos[1];
          odom.pose.pose.position.z = pos[2];
          odom.pose.pose.orientation.x = quat[0];
          odom.pose.pose.orientation.y = quat[1];
          odom.pose.pose.orientation.z = quat[2];
          odom.pose.pose.orientation.w = quat[3];
          const boost::array<double, 36> covariance = {{
                .1, 0, 0, 0, 0, 0,
                0, .1, 0, 0, 0, 0,
                0, 0, .15, 0, 0, 0,
                0, 0, 0, .01, 0, 0,
                0, 0, 0, 0, .01, 0,
                0, 0, 0, 0, 0, .01}};
          odom.pose.covariance = covariance;
          pubOdom.at(objectnum + k).publish(odom);
        }
      }
    }
    ROS_INFO_STREAM(ss_multiMarker.str());

    ROS_INFO("----   End marker output\n");

    for (j = 0; j < gARHandle->marker_num; j++) if (gARHandle->markerInfo[j].area > area) {
        area = gARHandle->markerInfo[j].area;
        biggestMarker = j;
      }

    if (area >= AR_AREA_MIN) {

      int imageProcMode;
      ARdouble pattRatio;
      ARdouble vertexUpright[4][2];

      // Reorder vertices based on dir.
      for (i = 0; i < 4; i++) {
        int dir = gARHandle->markerInfo[biggestMarker].dir;
        vertexUpright[i][0] = gARHandle->markerInfo[biggestMarker].vertex[(i + 4 - dir) % 4][0];
        vertexUpright[i][1] = gARHandle->markerInfo[biggestMarker].vertex[(i + 4 - dir) % 4][1];
      }
      arGetImageProcMode(gARHandle, &imageProcMode);
      arGetPattRatio(gARHandle, &pattRatio);
      if (matrixCodeType == AR_MATRIX_CODE_GLOBAL_ID) {
        gDrawPatternSize = 14;
        arPattGetImage2(imageProcMode, AR_MATRIX_CODE_DETECTION, gDrawPatternSize, gDrawPatternSize * AR_PATT_SAMPLE_FACTOR2,
                        imageCv.buff, gARHandle->xsize, gARHandle->ysize, gARHandle->arPixelFormat, &gCparamLT->paramLTf, vertexUpright, (ARdouble)14 / (ARdouble)(14 + 2), ext_patt);
      } else {
        gDrawPatternSize = matrixCodeType & AR_MATRIX_CODE_TYPE_SIZE_MASK;
        arPattGetImage2(imageProcMode, AR_MATRIX_CODE_DETECTION, gDrawPatternSize, gDrawPatternSize * AR_PATT_SAMPLE_FACTOR2,
                        imageCv.buff, gARHandle->xsize, gARHandle->ysize, gARHandle->arPixelFormat, &gCparamLT->paramLTf, vertexUpright, pattRatio, ext_patt);
      }
    } else {
      gDrawPatternSize = 0;
    }
  } else {
    gDrawPatternSize = 0;
  }

  // Tell GLUT the display has changed.
  if (gui)
    glutPostRedisplay();
}

//
//    This function is called on events when the visibility of the
//    GLUT window changes (including when it first becomes visible).
//
static void Visibility(int visible) {
  if (visible == GLUT_VISIBLE) {
    glutIdleFunc(mainLoop);
  } else {
    glutIdleFunc(NULL);
  }
}

//
//    This function is called when the
//    GLUT window is resized.
//
static void Reshape(int w, int h) {
  windowWidth = w;
  windowHeight = h;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  gViewport[0] = 0;
  gViewport[1] = 0;
  gViewport[2] = w;
  gViewport[3] = h;
  glViewport(0, 0, (GLsizei) w, (GLsizei) h);

  // Call through to anyone else who needs to know about window sizing here.
}

static void drawAxes() {
  GLfloat vertices[6][3] = {
    {0.0f, 0.0f, 0.0f}, {10.0f,  0.0f,  0.0f},
    {0.0f, 0.0f, 0.0f},  {0.0f, 10.0f,  0.0f},
    {0.0f, 0.0f, 0.0f},  {0.0f,  0.0f, 10.0f}
  };
  GLubyte colours[6][4] = {
    {255, 0, 0, 255}, {255, 0, 0, 255},
    {0, 255, 0, 255}, {0, 255, 0, 255},
    {0, 0, 255, 255}, {0, 0, 255, 255}
  };

  glVertexPointer(3, GL_FLOAT, 0, vertices);
  glColorPointer(4, GL_UNSIGNED_BYTE, 0, colours);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  glLineWidth(2.0f);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glDrawArrays(GL_LINES, 0, 6);
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);
}

//
// This function is called when the window needs redrawing.
//
static void Display(void) {
  ARdouble p[16];
  ARdouble m[16];
#ifdef ARDOUBLE_IS_FLOAT
  GLdouble p0[16];
  GLdouble m0[16];
#endif
  int i, j, k;
  GLfloat  w, bw, bh, vertices[6][2];
  GLubyte pixels[300];
  char text[256];
  GLdouble winX, winY, winZ;
  int showMErr[CHECK_ID_MULTIMARKERS_MAX];
  GLdouble MX[CHECK_ID_MULTIMARKERS_MAX];
  GLdouble MY[CHECK_ID_MULTIMARKERS_MAX];
  int pattDetectMode;
  AR_MATRIX_CODE_TYPE matrixCodeType;


  // Select correct buffer for this context.
  glDrawBuffer(GL_BACK);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.

  arglDispImage(gArglSettings);

  if (gMultiConfigCount) {
    arglCameraFrustumRH(&(gCparamLT->param), VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, p);
    glMatrixMode(GL_PROJECTION);
#ifdef ARDOUBLE_IS_FLOAT
    glLoadMatrixf(p);
#else
    glLoadMatrixd(p);
#endif
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_DEPTH_TEST);

    // If we have multi-configs, show their origin onscreen.
    for (k = 0; k < gMultiConfigCount; k++) {
      showMErr[k] = FALSE;
      if (gMultiConfigs[k]->prevF != 0) {
        arglCameraViewRH((const ARdouble (*)[4])gMultiConfigs[k]->trans, m, 1.0);
#ifdef ARDOUBLE_IS_FLOAT
        glLoadMatrixf(m);
#else
        glLoadMatrixd(m);
#endif
        drawAxes();
#ifdef ARDOUBLE_IS_FLOAT
        for (i = 0; i < 16; i++) m0[i] = (GLdouble)m[i];
        for (i = 0; i < 16; i++) p0[i] = (GLdouble)p[i];
        if (gluProject(0, 0, 0, m0, p0, gViewport, &winX, &winY, &winZ) == GL_TRUE)
#else
        if (gluProject(0, 0, 0, m, p, gViewport, &winX, &winY, &winZ) == GL_TRUE)
#endif
        {
          showMErr[k] = TRUE;
          MX[k] = winX; MY[k] = winY;
        }
      }

    } // for k
  }

  // Any 2D overlays go here.
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, (GLdouble)windowWidth, 0, (GLdouble)windowHeight, -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  arGetPatternDetectionMode(gARHandle, &pattDetectMode);
  arGetMatrixCodeType(gARHandle, &matrixCodeType);

  // For all markers, draw onscreen position.
  // Colour based on cutoffPhase.
  glLoadIdentity();
  glVertexPointer(2, GL_FLOAT, 0, vertices);
  glEnableClientState(GL_VERTEX_ARRAY);
  glLineWidth(2.0f);
  for (j = 0; j < gARHandle->marker_num; j++) {
    glColor3ubv(cutoffPhaseColours[gARHandle->markerInfo[j].cutoffPhase].colour);
    for (i = 0; i < 5; i++) {
      int dir = gARHandle->markerInfo[j].dir;
      vertices[i][0] = (float)gARHandle->markerInfo[j].vertex[(i + 4 - dir) % 4][0] * (float)windowWidth / (float)gARHandle->xsize;
      vertices[i][1] = ((float)gARHandle->ysize - (float)gARHandle->markerInfo[j].vertex[(i + 4 - dir) % 4][1]) * (float)windowHeight / (float)gARHandle->ysize;
    }
    vertices[i][0] = (float)gARHandle->markerInfo[j].pos[0] * (float)windowWidth / (float)gARHandle->xsize;
    vertices[i][1] = ((float)gARHandle->ysize - (float)gARHandle->markerInfo[j].pos[1]) * (float)windowHeight / (float)gARHandle->ysize;
    glDrawArrays(GL_LINE_STRIP, 0, 6);
    // For markers that have been identified, draw the ID number.
    if (gARHandle->markerInfo[j].id >= 0) {
      glColor3ub(255, 0, 0);
      if (matrixCodeType == AR_MATRIX_CODE_GLOBAL_ID && (pattDetectMode == AR_MATRIX_CODE_DETECTION || pattDetectMode == AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX || pattDetectMode == AR_TEMPLATE_MATCHING_MONO_AND_MATRIX)) snprintf(text, sizeof(text), "%lu (err=%d)", gARHandle->markerInfo[j].globalID, gARHandle->markerInfo[j].errorCorrected);
      else snprintf(text, sizeof(text), "%d", gARHandle->markerInfo[j].id);
      print(text, (float)gARHandle->markerInfo[j].pos[0] * (float)windowWidth / (float)gARHandle->xsize, ((float)gARHandle->ysize - (float)gARHandle->markerInfo[j].pos[1]) * (float)windowHeight / (float)gARHandle->ysize, 0, 0);
    }
  }
  glDisableClientState(GL_VERTEX_ARRAY);

  // For matrix mode, draw the pattern image of the largest marker.
  if (gDrawPatternSize) {
    int zoom = 4;
    glRasterPos2f((float)(windowWidth - gDrawPatternSize * zoom) - 4.0f, (float)(gDrawPatternSize * zoom) + 4.0f);
    glPixelZoom((float)zoom, (float) - zoom);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glDrawPixels(gDrawPatternSize, gDrawPatternSize, GL_LUMINANCE, GL_UNSIGNED_BYTE, ext_patt);
    glPixelZoom(1.0f, 1.0f);
  }


  // Draw error value for multimarker pose.
  for (k = 0; k < gMultiConfigCount; k++) {
    if (showMErr[k]) {
      snprintf(text, sizeof(text), "err=%0.3f", gMultiErrs[k]);
      print(text, MX[k], MY[k], 0, 0);
    }
  }

  //
  // Draw help text and mode.
  //
  glLoadIdentity();
  if (gShowMode) {
    printMode();
  }
  if (gShowHelp) {
    if (gShowHelp == 1) {
      printHelpKeys();
    } else if (gShowHelp == 2) {
      bw = 0.0f;
      for (i = 0; i < AR_MARKER_INFO_CUTOFF_PHASE_DESCRIPTION_COUNT; i++) {
        w = (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (unsigned char *)arMarkerInfoCutoffPhaseDescriptions[cutoffPhaseColours[i].cutoffPhase]);
        if (w > bw) bw = w;
      }
      bw += 12.0f; // Space for color block.
      bh = AR_MARKER_INFO_CUTOFF_PHASE_DESCRIPTION_COUNT * 10.0f /* character height */ + (AR_MARKER_INFO_CUTOFF_PHASE_DESCRIPTION_COUNT - 1) * 2.0f /* line spacing */;
      drawBackground(bw, bh, 2.0f, 2.0f);

      // Draw the colour block and text, line by line.
      for (i = 0; i < AR_MARKER_INFO_CUTOFF_PHASE_DESCRIPTION_COUNT; i++) {
        for (j = 0; j < 300; j += 3) {
          pixels[j    ] = cutoffPhaseColours[i].colour[0];
          pixels[j + 1] = cutoffPhaseColours[i].colour[1];
          pixels[j + 2] = cutoffPhaseColours[i].colour[2];
        }
        glRasterPos2f(2.0f, (AR_MARKER_INFO_CUTOFF_PHASE_DESCRIPTION_COUNT - 1 - i) * 12.0f + 2.0f);
        glPixelZoom(1.0f, 1.0f);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glDrawPixels(10, 10, GL_RGB, GL_UNSIGNED_BYTE, pixels);
        print(arMarkerInfoCutoffPhaseDescriptions[cutoffPhaseColours[i].cutoffPhase], 14.0f, (AR_MARKER_INFO_CUTOFF_PHASE_DESCRIPTION_COUNT - 1 - i) * 12.0f + 2.0f, 0, 0);
      }
    }
  }

  glutSwapBuffers();
}

//
// The following functions provide the onscreen help text and mode info.
//

static void print(const char *text, const float x, const float y, int calculateXFromRightEdge, int calculateYFromTopEdge) {
  int i, len;
  GLfloat x0, y0;

  if (!text) return;

  if (calculateXFromRightEdge) {
    x0 = windowWidth - x - (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (const unsigned char *)text);
  } else {
    x0 = x;
  }
  if (calculateYFromTopEdge) {
    y0 = windowHeight - y - 10.0f;
  } else {
    y0 = y;
  }
  glRasterPos2f(x0, y0);

  len = (int)strlen(text);
  for (i = 0; i < len; i++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
}

static void drawBackground(const float width, const float height, const float x, const float y) {
  GLfloat vertices[4][2];

  vertices[0][0] = x; vertices[0][1] = y;
  vertices[1][0] = width + x; vertices[1][1] = y;
  vertices[2][0] = width + x; vertices[2][1] = height + y;
  vertices[3][0] = x; vertices[3][1] = height + y;
  glLoadIdentity();
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glVertexPointer(2, GL_FLOAT, 0, vertices);
  glEnableClientState(GL_VERTEX_ARRAY);
  glColor4f(0.0f, 0.0f, 0.0f, 0.5f);    // 50% transparent black.
  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Opaque white.
  //glLineWidth(1.0f);
  //glDrawArrays(GL_LINE_LOOP, 0, 4);
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisable(GL_BLEND);
}

static void printHelpKeys() {
  int i;
  GLfloat  w, bw, bh;
  const char *helpText[] = {
    "Keys:\n",
    " ? or /        Show/hide this help / marker cutoff phase key.",
    " q or [esc]    Quit program.",
    " d             Activate / deactivate debug mode.",
    " m             Toggle display of mode info.",
    " a             Toggle between available threshold modes.",
    " - and +       Switch to manual threshold mode, and adjust threshhold up/down by 5.",
    " x             Change image processing mode.",
    " c             Calulcate frame rate.",
    " r             Toggle robust multi-marker mode on/off.",
  };
#define helpTextLineCount (sizeof(helpText)/sizeof(char *))

  bw = 0.0f;
  for (i = 0; i < helpTextLineCount; i++) {
    w = (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (unsigned char *)helpText[i]);
    if (w > bw) bw = w;
  }
  bh = helpTextLineCount * 10.0f /* character height */ + (helpTextLineCount - 1) * 2.0f /* line spacing */;
  drawBackground(bw, bh, 2.0f, 2.0f);

  for (i = 0; i < helpTextLineCount; i++) print(helpText[i], 2.0f, (helpTextLineCount - 1 - i) * 12.0f + 2.0f, 0, 0);;
}

static void printMode() {
  int len, thresh, line, mode, xsize = 1000, ysize = 1000, textPatternCount;
  AR_LABELING_THRESH_MODE threshMode;
  ARdouble tempF;
  char text[256];
  std::string text_s;

  glColor3ub(255, 255, 255);
  line = 1;

  // Image size and processing mode.
//  arVideoGetSize(&xsize, &ysize);
  arGetImageProcMode(gARHandle, &mode);
  if (mode == AR_IMAGE_PROC_FRAME_IMAGE) text_s = "full frame";
  else text_s = "even field only";
  snprintf(text, sizeof(text), "Processing %dx%d video frames %s", xsize, ysize, text_s.c_str());
  print(text, 2.0f,  (line - 1) * 12.0f + 2.0f, 0, 1);
  line++;

  // Threshold mode, and threshold, if applicable.
  arGetLabelingThreshMode(gARHandle, &threshMode);
  switch (threshMode) {
  case AR_LABELING_THRESH_MODE_MANUAL: text_s = "MANUAL"; break;
  case AR_LABELING_THRESH_MODE_AUTO_MEDIAN: text_s = "AUTO_MEDIAN"; break;
  case AR_LABELING_THRESH_MODE_AUTO_OTSU: text_s = "AUTO_OTSU"; break;
  case AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE: text_s = "AUTO_ADAPTIVE"; break;
  case AR_LABELING_THRESH_MODE_AUTO_BRACKETING: text_s = "AUTO_BRACKETING"; break;
  default: text_s = "UNKNOWN"; break;
  }
  snprintf(text, sizeof(text), "Threshold mode: %s", text_s.c_str());
  if (threshMode != AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE) {
    arGetLabelingThresh(gARHandle, &thresh);
    len = (int)strlen(text);
    snprintf(text + len, sizeof(text) - len, ", thresh=%d", thresh);
  }
  print(text, 2.0f,  (line - 1) * 12.0f + 2.0f, 0, 1);
  line++;

  // Border size, image processing mode, pattern detection mode.
  arGetBorderSize(gARHandle, &tempF);
  snprintf(text, sizeof(text), "Border: %0.2f%%", tempF * 100.0);
  arGetPatternDetectionMode(gARHandle, &mode);
  textPatternCount = 0;
  switch (mode) {
  case AR_TEMPLATE_MATCHING_COLOR: text_s = "Colour template (pattern)"; break;
  case AR_TEMPLATE_MATCHING_MONO: text_s = "Mono template (pattern)"; break;
  case AR_MATRIX_CODE_DETECTION: text_s = "Matrix (barcode)"; textPatternCount = -1; break;
  case AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX: text_s = "Colour template + Matrix (2 pass, pattern + barcode)"; break;
  case AR_TEMPLATE_MATCHING_MONO_AND_MATRIX: text_s = "Mono template + Matrix (2 pass, pattern + barcode "; break;
  default: text_s = "UNKNOWN"; textPatternCount = -1; break;
  }
  if (textPatternCount != -1) textPatternCount = gARPattHandle->patt_num;
  len = (int)strlen(text);
  if (textPatternCount != -1) snprintf(text + len, sizeof(text) - len, ", Pattern detection mode: %s, %d patterns loaded", text_s.c_str(), textPatternCount);
  else snprintf(text + len, sizeof(text) - len, ", Pattern detection mode: %s", text_s.c_str());
  print(text, 2.0f,  (line - 1) * 12.0f + 2.0f, 0, 1);
  line++;

  // Robust mode.
  if (gMultiConfigCount) {
    snprintf(text, sizeof(text), "Robust multi-marker pose estimation %s", (gRobustFlag ? "ON" : "OFF"));
    if (gRobustFlag) {
      icpGetInlierProbability(gAR3DHandle->icpHandle, &tempF);
      len = (int)strlen(text);
      snprintf(text + len, sizeof(text) - len, ", inliner prob. %0.1f%%", tempF * 100.0f);
    }
    print(text, 2.0f,  (line - 1) * 12.0f + 2.0f, 0, 1);
    line++;
  }

  // Window size.
  snprintf(text, sizeof(text), "Drawing into %dx%d window", windowWidth, windowHeight);
  print(text, 2.0f,  (line - 1) * 12.0f + 2.0f, 0, 1);
  line++;

}
