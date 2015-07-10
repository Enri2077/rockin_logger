#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
// #include <audio_common_msgs/AudioData.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h> // Enrico: I commented this line because pcl_conversions.h is not available in groovy

using namespace std;

#define NODE_NAME "rockin_logger"


// robot topics
#define SCAN_LEFT_TOPIC "/scan_left"
#define SCAN_RIGHT_TOPIC "/scan_right"

// robot frames
#define MAP_FRAME  "/map"
#define BASE_FRAME  "/roamfree"


// output RoCKIn log topics
#define ROCKIN_ROBOTPOSE "/rockin/robot_pose"
#define ROCKIN_MARKERPOSE "/rockin/marker_pose"
#define ROCKIN_SCAN_LEFT "/rockin/scan_0"
#define ROCKIN_SCAN_RIGHT "/rockin/scan_1"
#define ROCKIN_IMAGE "/rockin/image"
#define ROCKIN_POINTCLOUD "/rockin/pointcloud"
#define ROCKIN_COMMAND "/rockin/command"
#define ROCKIN_AUDIO "/rockin/audio"



void laserLeftCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void laserRightCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void loggerTimerCallback(const ros::TimerEvent& msg);
void imageTimerCallback(const ros::TimerEvent& msg);
void commandTimerCallback(const ros::TimerEvent& msg);

struct TLogger {
  tf::TransformListener* listener;
  image_transport::ImageTransport *imagetr;
  string image_file, pcd_file, robot_name;
  sensor_msgs::LaserScan scan_left_msg, scan_right_msg;
  cv::Mat image;
  // pcl::PCLPointCloud2 pointcloud; // Enrico: I commented this line because pcl_conversions.h is not available in groovy
  // sensor_msgs::PointCloud2 pointcloud_msg; // Enrico: I commented this line because pcl_conversions.h is not available in groovy
  // audio_common_msgs::AudioData audio_msg;
  
  string map_frame, base_frame, scan_left_topic, scan_right_topic;
  
  ros::Subscriber laserLeftSub, laserRightSub;
  ros::Publisher rposePub, mposePub, scanLeftPub, scanRightPub, commandPub, pointcloudPub, audioPub;
  image_transport::Publisher imagePub; 

  ros::Timer logger_timer, image_timer, command_timer;

  
  ros::Time now;

  TLogger() {
    listener=NULL;
    
    image_file = "default.png"; pcd_file = "default.pcd"; robot_name = "lurch";
  }
  
  void init(ros::NodeHandle &n) {
    setTopicAndFrameNames();
    initSubsAndPubs(n);
    loadData();  
  }
  
  void initSubsAndPubs(ros::NodeHandle &n) {

    listener = new tf::TransformListener();
    imagetr = new image_transport::ImageTransport(n);
    
    laserLeftSub = n.subscribe(scan_left_topic, 10, laserLeftCallback);
    laserRightSub = n.subscribe(scan_right_topic, 10, laserRightCallback);

    rposePub = n.advertise<geometry_msgs::PoseStamped>(ROCKIN_ROBOTPOSE, 10);
    mposePub = n.advertise<geometry_msgs::PoseStamped>(ROCKIN_MARKERPOSE, 10);
    scanLeftPub = n.advertise<sensor_msgs::LaserScan>(ROCKIN_SCAN_LEFT, 10);
    scanRightPub = n.advertise<sensor_msgs::LaserScan>(ROCKIN_SCAN_RIGHT, 10);
    imagePub = imagetr->advertise(ROCKIN_IMAGE, 1);
    commandPub = n.advertise<std_msgs::String>(ROCKIN_COMMAND, 1);
    // pointcloudPub = n.advertise<pcl::PCLPointCloud2> (ROCKIN_POINTCLOUD, 1); // Enrico: I commented this line because pcl_conversions.h is not available in groovy
    // audio_capture node is used to capture audio
    //audioPub = n.advertise<audio_common_msgs::AudioData>(ROCKIN_AUDIO, 1);
    
    logger_timer = n.createTimer(ros::Duration(0.1), loggerTimerCallback);  // 10 Hz
    image_timer = n.createTimer(ros::Duration(5.0), imageTimerCallback);    // 1/5 Hz
    command_timer = n.createTimer(ros::Duration(7.0), commandTimerCallback);    // 1/7 Hz
    
    loadData();
    
  }

  void loadData() {
    /* image.create(320, 240, CV_8UC3);
    cv::circle(image, cv::Point(120, 160), 40, CV_RGB(255,0,0));
    cv::circle(image, cv::Point(120, 160), 60, CV_RGB(0,255,0));
    cv::circle(image, cv::Point(120, 160), 80, CV_RGB(0,0,255)); */
    image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
    
    // pcl::io::loadPCDFile (pcd_file, pointcloud); // Enrico: I commented this line because pcl_conversions.h is not available in groovy
    // pointcloud.header.frame_id = base_frame; // In general it should be in camera frame  // Enrico: I commented this line because pcl_conversions.h is not available in groovy
  }
  
  void  setTopicAndFrameNames() {
    scan_left_topic = SCAN_LEFT_TOPIC;
    scan_right_topic = SCAN_RIGHT_TOPIC;
    base_frame = BASE_FRAME;
    map_frame = MAP_FRAME;
    if (robot_name!="") {
      //scan_left_topic = "/" + robot_name + SCAN_LEFT_TOPIC;
      //scan_right_topic = "/" + robot_name + SCAN_RIGHT_TOPIC;
      base_frame = "/" + robot_name + BASE_FRAME;
    }      
 
  }
  
};

static TLogger logger;

void laserLeftCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  logger.scan_left_msg = *msg;
}

void laserRightCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  logger.scan_right_msg = *msg;
}


bool getRobotPose(double &x, double &y, double &th_rad)
{
    string src_frame = logger.map_frame;
    string dest_frame = logger.base_frame;
    tf::StampedTransform transform;
    try {
      logger.listener->waitForTransform(src_frame, dest_frame, ros::Time(0), ros::Duration(3));
      logger.listener->lookupTransform(src_frame, dest_frame, ros::Time(0), transform);
      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
      th_rad = tf::getYaw(transform.getRotation());
    }
    catch(tf::TransformException ex) {
        th_rad = 3*M_PI;
        ROS_ERROR("Error in tf trasnform %s -> %s\n",src_frame.c_str(), dest_frame.c_str());
	ROS_ERROR("%s", ex.what());
        return false;
    }
    return true;
}


// Data logger
void loggerTimerCallback(const ros::TimerEvent& msg)
{
  // ROS_INFO("Publishing info...");
  logger.now = ros::Time::now();
  double x,y,th_rad;
  geometry_msgs::PoseStamped rpose,mpose;

  bool r = getRobotPose(x, y, th_rad);
  if (r) {
    rpose.header.stamp = logger.now;
    rpose.header.frame_id = MAP_FRAME;
    rpose.pose.position.x = x; rpose.pose.position.y = y; rpose.pose.position.z = 0;
    tf::Quaternion q; q.setRPY(0, 0, th_rad);    
    tf::quaternionTFToMsg(q, rpose.pose.orientation);
    logger.rposePub.publish(rpose);
    mpose = rpose; mpose.pose.position.z += 1.0;
    q.setRPY(0.3, -0.7, th_rad);
    tf::quaternionTFToMsg(q, mpose.pose.orientation);
    logger.mposePub.publish(mpose);
  }
  logger.scanLeftPub.publish(logger.scan_left_msg);
  logger.scanRightPub.publish(logger.scan_right_msg);

}

// Image logger
void imageTimerCallback(const ros::TimerEvent& msg)
{
  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", logger.image).toImageMsg();
  image_msg->header.stamp = logger.now;
  logger.imagePub.publish(image_msg);
  
  
  // logger.pointcloud.header.stamp =  pcl_conversions::toPCL(logger.now); // Enrico: I commented this line because pcl_conversions.h is not available in groovy
  // logger.pointcloudPub.publish(logger.pointcloud); // Enrico: I commented this line because pcl_conversions.h is not available in groovy
}

// Command logger
void commandTimerCallback(const ros::TimerEvent& msg)
{
  std_msgs::String s_msg; s_msg.data = "go to the kitchen";
  logger.commandPub.publish(s_msg);
  
  // logger.audioPub.publish(logger.audio_msg);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n, np("~");

  np.getParam("image_file", logger.image_file);
  np.getParam("pcd_file", logger.pcd_file);
  np.getParam("robot_name", logger.robot_name);
  
  logger.init(n);

  ros::spin();

  return 0;
}


