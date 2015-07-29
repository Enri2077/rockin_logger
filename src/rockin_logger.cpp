#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

#define NODE_NAME			"rockin_logger"

// output RoCKIn log topics
#define ROCKIN_ROBOTPOSE	"/robot_pose"
#define ROCKIN_MARKERPOSE	"/marker_pose"
#define ROCKIN_SCAN_0		"/scan_0"		// TO BE REPLICATED
#define ROCKIN_SCAN_1		"/scan_1"		// TO BE REPLICATED
#define ROCKIN_IMAGE		"/image"
#define ROCKIN_POINTCLOUD	"/pointcloud"
#define ROCKIN_COMMAND		"/command"



void laserCallback0(const sensor_msgs::LaserScan::ConstPtr& msg);	// TO BE REPLICATED
void laserCallback1(const sensor_msgs::LaserScan::ConstPtr& msg);	// TO BE REPLICATED

void loggerTimerCallback(const ros::TimerEvent& msg);
void imageTimerCallback(const ros::TimerEvent& msg);
void commandTimerCallback(const ros::TimerEvent& msg);

struct TLogger{
	tf::TransformListener* listener;
	image_transport::ImageTransport	*imagetr;
	cv::Mat image;

	pcl::PCLPointCloud2 pointcloud;
	sensor_msgs::PointCloud2 pointcloud_msg;

	/**	Parameters provided by launch file,
	  *	the only customization here should be the replication for the topics 
	  *	(i.e. when you need to publish 2 scan topics: scan_topic_0 and scan_topic_1)
	**/
	string image_file, pcd_file, team_name, map_frame, base_frame, topic_prefix;
	string scan_topic_0;				// TO BE REPLICATED
	string scan_topic_1;				// TO BE REPLICATED
	
	sensor_msgs::LaserScan scan_msg_0;	// TO BE REPLICATED
	ros::Subscriber laserSub0;			// TO BE REPLICATED
	
	sensor_msgs::LaserScan scan_msg_1;	// TO BE REPLICATED
	ros::Subscriber laserSub1;			// TO BE REPLICATED
	
	ros::Publisher scanPub0;			// TO BE REPLICATED
	ros::Publisher scanPub1;			// TO BE REPLICATED
	ros::Publisher rposePub, mposePub, commandPub, pointcloudPub, audioPub;
	image_transport::Publisher imagePub;
	
	double translation_x, translation_y, translation_z;
	
	ros::Timer logger_timer, image_timer, command_timer;
	
	ros::Time now;

	TLogger(){
		listener=NULL;
	}

	void init(ros::NodeHandle &n){
		initSubsAndPubs(n);
		loadData();
	}

	void initSubsAndPubs(ros::NodeHandle &n){
		listener = new tf::TransformListener();
		imagetr = new image_transport::ImageTransport(n);

		laserSub0 = n.subscribe(scan_topic_0, 10, laserCallback0);			// TO BE REPLICATED
		laserSub1 = n.subscribe(scan_topic_1, 10, laserCallback1);			// TO BE REPLICATED

		scanPub0 = n.advertise<sensor_msgs::LaserScan>(topic_prefix+ROCKIN_SCAN_0, 10);	// TO BE REPLICATED
		scanPub1 = n.advertise<sensor_msgs::LaserScan>(topic_prefix+ROCKIN_SCAN_1, 10);	// TO BE REPLICATED
		rposePub = n.advertise<geometry_msgs::PoseStamped>(topic_prefix+ROCKIN_ROBOTPOSE, 10);
		mposePub = n.advertise<geometry_msgs::PoseStamped>(topic_prefix+ROCKIN_MARKERPOSE, 10);
		imagePub = imagetr->advertise(topic_prefix+ROCKIN_IMAGE, 1);
		commandPub = n.advertise<std_msgs::String>(topic_prefix+ROCKIN_COMMAND, 1);
		pointcloudPub = n.advertise<pcl::PCLPointCloud2> (topic_prefix+ROCKIN_POINTCLOUD, 1);
		
		logger_timer = n.createTimer(ros::Duration(0.1), loggerTimerCallback);  // 10 Hz
		image_timer = n.createTimer(ros::Duration(5.0), imageTimerCallback);    // 1/5 Hz
		command_timer = n.createTimer(ros::Duration(7.0), commandTimerCallback);    // 1/7 Hz
		
		loadData();
	}
	
	void loadData(){
		/* image.create(320, 240, CV_8UC3);
		cv::circle(image, cv::Point(120, 160), 40, CV_RGB(255,0,0));
		cv::circle(image, cv::Point(120, 160), 60, CV_RGB(0,255,0));
		cv::circle(image, cv::Point(120, 160), 80, CV_RGB(0,0,255)); */
		image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);

		pcl::io::loadPCDFile (pcd_file, pointcloud);
		pointcloud.header.frame_id = base_frame; // In general it should be in camera frame
	}
	
};

static TLogger logger;

void laserCallback0(const sensor_msgs::LaserScan::ConstPtr& msg){	// TO BE REPLICATED
	logger.scan_msg_0 = *msg;
}
void laserCallback1(const sensor_msgs::LaserScan::ConstPtr& msg){	// TO BE REPLICATED
	logger.scan_msg_1 = *msg;
}


bool getRobotPose(double &x, double &y, double &th_rad){
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
void loggerTimerCallback(const ros::TimerEvent& msg){
	// ROS_INFO("Publishing info...");
	logger.now = ros::Time::now();
	double x, y, th_rad;
	geometry_msgs::PoseStamped rpose, mpose;
	bool r = getRobotPose(x, y, th_rad);
	
	if (r){
		rpose.header.stamp = logger.now;
		rpose.header.frame_id = logger.map_frame;
		tf::Quaternion q;
		
		// robot position and orientation
		rpose.pose.position.x = x;
		rpose.pose.position.y = y;
		rpose.pose.position.z = 0;		
		q.setRPY(0, 0, th_rad);
		
		// publish robot pose
		tf::quaternionTFToMsg(q, rpose.pose.orientation);
		logger.rposePub.publish(rpose);
		
		// marker position and orientation
		mpose = rpose;
		mpose.pose.position.x += logger.translation_x;
		mpose.pose.position.y += logger.translation_y;
		mpose.pose.position.z += logger.translation_z;
		q.setRPY(0, 0, th_rad);								// the marker's direction is the same as the robot
		
		// publish marker pose
		tf::quaternionTFToMsg(q, mpose.pose.orientation);
		logger.mposePub.publish(mpose);
	}
	
	logger.scanPub0.publish(logger.scan_msg_0);		// TO BE REPLICATED
	logger.scanPub1.publish(logger.scan_msg_1);		// TO BE REPLICATED
}

// Image logger
void imageTimerCallback(const ros::TimerEvent& msg){
	sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", logger.image).toImageMsg();
	image_msg->header.stamp = logger.now;
	logger.imagePub.publish(image_msg);


	logger.pointcloud.header.stamp =  pcl_conversions::toPCL(logger.now);
	logger.pointcloudPub.publish(logger.pointcloud);
}

// Command logger
void commandTimerCallback(const ros::TimerEvent& msg){
	std_msgs::String s_msg; s_msg.data = "go to the kitchen";
	logger.commandPub.publish(s_msg);
}


int main(int argc, char **argv){
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n, np("~");
	
	np.getParam("team_name",		logger.team_name);

	np.getParam("image_file",		logger.image_file);
	np.getParam("pcd_file",			logger.pcd_file);
	
	np.getParam("scan_topic_0",		logger.scan_topic_0);		// TO BE REPLICATED
	np.getParam("scan_topic_1",		logger.scan_topic_1);		// TO BE REPLICATED
	
	np.getParam("map_frame",		logger.map_frame);
	np.getParam("base_frame",		logger.base_frame);
	
	np.getParam("translation_x",	logger.translation_x);
	np.getParam("translation_y",	logger.translation_y);
	np.getParam("translation_z",	logger.translation_z);
	
	logger.topic_prefix = "/rockin/" + logger.team_name;
	
	logger.init(n);
	ros::spin();
	return 0;
}

