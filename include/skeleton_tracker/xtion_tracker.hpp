/**
 * \ref xtion_tracker.hpp
 *
 *  \date 11/12/2015
 *  author Muhannad Alomari
 *  author Yiannis G.
 *  author Alessio Levratti
 *  version 1.0
 *  bug
 *  copyright GNU Public License.
 */

#ifndef XTION_TRACKER_HPP_
#define XTION_TRACKER_HPP_

// ROS Dependencies
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include "NiTE.h"
#include <openni2_camera/openni2_device.h>
#include <openni2/OniCTypes.h>
#include <openni2/OpenNI.h>
#include <openni2/OniCEnums.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <skeleton_tracker/user_IDs.h>
#include "skeleton_tracker/skeleton_tracker_state.h"
#include "skeleton_tracker/joint_message.h"
#include "skeleton_tracker/skeleton_message.h"
#include <geometry_msgs/Pose.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
//
#include <yaml-cpp-0.3/yaml.h>
#include <fstream>

#ifndef ALPHA
#define ALPHA 1/256
#endif

#define MAX_USERS 100

#define USER_MESSAGE(msg) \
        {printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

/// Joint map
typedef std::map<std::string, nite::SkeletonJoint> JointMap;

/// yaml stuff
struct Vec5 {
   float x[5] = { 0 };
};
struct Vec9 {
   float x[9] = { 0 };
};
struct Vec12 {
   float x[12] = { 0 };
};
void operator >> (const YAML_0_3::Node& node, Vec5& v) {
    for(int i=0;i<5;i++)
      node[i] >> v.x[i];
}
void operator >> (const YAML_0_3::Node& node, Vec9& v) {
    for(int i=0;i<9;i++)
      node[i] >> v.x[i];
}
void operator >> (const YAML_0_3::Node& node, Vec12& v) {
    for(int i=0;i<12;i++)
      node[i] >> v.x[i];
}

/// camera parameters
double fx = 525.0;    // default values
double fy = 525.0;    // default values
double cx = 319.5;    // default values
double cy = 239.5;    // default values

::skeleton_tracker::joint_message generate_joint_message(std::string joint_name, nite::JointType nite_type, JointMap &named_j, ::geometry_msgs::Pose &p, const nite::UserData& user)
{
  named_j[joint_name] = (user.getSkeleton().getJoint(nite_type));

  p.position.x = named_j[joint_name].getPosition().x/1000;
  p.position.y = named_j[joint_name].getPosition().y/-1000;
  p.position.z = named_j[joint_name].getPosition().z/1000;

  ::skeleton_tracker::joint_message msg;
  msg.name = joint_name;
  msg.pose = p;
  msg.confidence = named_j[joint_name].getPositionConfidence();
  return msg;
}

void generate_joint_coordinates(std::string joint_name,int joint_id, int (&X)[15], int (&Y)[15],  JointMap &named_j)
{
  X[joint_id]=int(named_j[joint_name].getPosition().x*fx/named_j[joint_name].getPosition().z +cx);
  Y[joint_id]=int(named_j[joint_name].getPosition().y*fy/named_j[joint_name].getPosition().z*-1 +cy);
}

/**
 * Union for color definition
 */
typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

/**
 * Class \ref xtion_tracker. This class can track the skeleton of people and returns joints as a TF stream,
 *  while publishing the video stream and the point cloud captured by an ASUS Xtion Pro Live.
 */
void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
   std::cout << msg << std::endl;
}

class xtion_tracker
{
public:
  /**
   * Constructor
   */
  xtion_tracker() :
      it_(nh_)
  {

    // Get some parameters from the server
    ros::NodeHandle pnh("~");
    if (!pnh.getParam("camera_calibration", camera_calibration))
    {
      ROS_FATAL("camera_calibration not found on Param Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }
    if (!pnh.getParam("camera", camera))
    {
      ROS_FATAL("camera not found on Parameter Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }

    if (!pnh.getParam("rgb_frame", rgb_frame))
    {
      ROS_FATAL("rgb_frame not found on Parameter Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }

    if (!pnh.getParam("depth_frame", depth_frame))
    {
      ROS_FATAL("depth_frame not found on Parameter Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }

    // Initialize OpenNI
    if (openni::OpenNI::initialize() != openni::STATUS_OK)
    {
      ROS_FATAL("OpenNI initial error");
      ros::shutdown();
      return;
    }

    // Open the device
    if (devDevice_.open(openni::ANY_DEVICE) != openni::STATUS_OK)
    {
      ROS_FATAL("Can't Open Device");
      ros::shutdown();
      return;
    }
    ROS_INFO("Device opened");

    // Initialize the tracker
    nite::NiTE::initialize();

    // Set the depth mode
    if (depthStream_.create(devDevice_, openni::SENSOR_DEPTH) == openni::STATUS_OK)
    {
      depthMode_ = depthStream_.getSensorInfo().getSupportedVideoModes()[4];
      ROS_INFO("The wished depth mode is %d x %d at %d FPS. Pixel format %d", depthMode_.getResolutionX(),
               depthMode_.getResolutionY(), depthMode_.getFps(), depthMode_.getPixelFormat());
      if (depthStream_.setVideoMode(depthMode_) != openni::STATUS_OK)
      {
        ROS_ERROR("Can't apply depth-videomode");
        depthMode_ = depthStream_.getVideoMode();
        ROS_INFO("The depth mode is set to %d x %d at %d FPS. Pixel format %d", depthMode_.getResolutionX(),
                 depthMode_.getResolutionY(), depthMode_.getFps(), depthMode_.getPixelFormat());
      }

      depthStream_.setMirroringEnabled(false);
    }
    else
    {
      ROS_FATAL("Can't create depth stream on device");
      ros::shutdown();
      return;
    }

    // Set the video mode
    if (vsColorStream_.create(devDevice_, openni::SENSOR_COLOR) == openni::STATUS_OK)
    {
      // set video mode
      mMode_.setResolution(640, 480);
      mMode_.setFps(30);
      mMode_.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
      ROS_INFO("The wished video mode is %d x %d at %d FPS", mMode_.getResolutionX(), mMode_.getResolutionY(),
               mMode_.getFps());

      if (vsColorStream_.setVideoMode(mMode_) != openni::STATUS_OK)
      {
        ROS_ERROR("Can't apply videomode\n");
        ROS_INFO("The video mode is set to %d x %d at %d FPS", mMode_.getResolutionX(), mMode_.getResolutionY(),
                 mMode_.getFps());
        mMode_ = vsColorStream_.getVideoMode();
      }

      // image registration
      if (devDevice_.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
      {
        devDevice_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      }
      vsColorStream_.setMirroringEnabled(true);
    }
    else
    {
      ROS_FATAL("Can't create color stream on device: ");
      ros::shutdown();
      return;
    }

    niteRc_ = userTracker_.create();
    if (niteRc_ != nite::STATUS_OK)
    {
      ROS_FATAL("Couldn't create user tracker");
      ros::shutdown();
      return;
    }

    // Start the RGB video stream and the depth video stream
    vsColorStream_.start();
    depthStream_.start();

    // Initialize the image publisher
    imagePub_ = it_.advertise("/"+camera+"/rgb/image_raw", 1);
    imageSKPub_ = it_.advertise("/"+camera+"/rgb/sk_tracks", 1);
    // Initialize the depth image publisher
    depthPub_ = it_.advertise("/"+camera+"/depth/image_raw", 1);
    // Initialize the users IDs publisher
    userPub_ = nh_.advertise<skeleton_tracker::user_IDs>("/people", 1);
    sub = nh_.subscribe("/image_calib", 1000, chatterCallback);

    // read yaml files
    std::ifstream fin("/home/lucie02/.ros/camera_info/rgb_PS1080_PrimeSense.yaml");
    if (fin.good())
    {
        YAML_0_3::Parser parser(fin);
        YAML_0_3::Node config;

        Vec9 camera_param;
        Vec5 distortion;
        Vec9 rectification;
        Vec12 projection;

        parser.GetNextDocument(config);

        config["image_width"] >> width;
        config["image_height"] >> height;
        config["camera_matrix"]["data"] >> camera_param;
        config["distortion_coefficients"]["data"] >> distortion;
        config["rectification_matrix"]["data"] >> rectification;
        config["projection_matrix"]["data"] >> projection;

        for (std::size_t i=0;i<9;i++){K.push_back(camera_param.x[i]);}
        for (std::size_t i=0;i<9;i++){R.push_back(rectification.x[i]);}
        for (std::size_t i=0;i<5;i++){D.push_back(distortion.x[i]);}
        for (std::size_t i=0;i<12;i++){P.push_back(projection.x[i]);}
        fx=K[0]; fy=K[4]; cx=K[2]; cy=K[5];

        ROS_INFO("reading camera calibration file...");
        std::cout<<"Camera matrix: "<<"fx:"<<fx<<" "<<"fy:"<<fy<<" "<<"cx:"<<cx<<" "<<"cy:"<<cy<<std::endl;
        std::cout<<"Rectification coefficients: ";
        for(std::size_t i=0;i<R.size();i++)
            std::cout<<R[i]<<" ";
        std::cout<<std::endl;
        std::cout<<"Distortion coefficients: ";
        for(std::size_t i=0;i<D.size();i++)
            std::cout<<D[i]<<" ";
        std::cout<<std::endl;
        std::cout<<"Projection matrix: ";
        for(std::size_t i=0;i<P.size();i++)
            std::cout<<P[i]<<" ";
        read_yaml_success = 1;
        std::cout<<std::endl;
    }
    else{
        ROS_INFO("camera calibration file could not be opened.");
    }


    // Initialize both the Camera Info publishers
    depthInfoPub_ = nh_.advertise<sensor_msgs::CameraInfo>("/"+camera+"/depth/camera_info", 1);
    rgbInfoPub_ = nh_.advertise<sensor_msgs::CameraInfo>("/"+camera+"/rgb/camera_info", 1);
    incremental_msg_pub_ = nh_.advertise<skeleton_tracker::skeleton_message>("/skeleton_data/incremental", 1);
    // Initialize the skeleton state publisher
    skeleton_state_pub_ = nh_.advertise<skeleton_tracker::skeleton_tracker_state>("skeleton_data/state", 10);
  }
  /**
   * Destructor
   */
  ~xtion_tracker()
  {
    nite::NiTE::shutdown();
  }

  /**
   * Spinner!!!
   */
  void spinner()
  {
    // Broadcast the RGB video
    this->broadcastVideo();

    // Broadcast the depth image
    this->getDepth();

    // Broadcast the joint frames (if they exist)
    this->getSkeleton(); //uncomment

  }

private:

  template<typename T>
  std::string num_to_str(T num) {
      std::stringstream ss;
      ss << std::setprecision(20) << num;
      return ss.str();
  }

  std::string generateUUID(std::string time, long id) {
    boost::uuids::name_generator gen(dns_namespace_uuid);
    time += num_to_str<long>(id);

    return num_to_str<boost::uuids::uuid>(gen(time.c_str()));
  }

   /**
   * RGB Video broadcaster
   */
  void broadcastVideo()
  {

    if (vsColorStream_.readFrame(&vfColorFrame_) == openni::STATUS_OK)
    {
      // convert data to OpenCV format
      const cv::Mat mImageRGB(vfColorFrame_.getHeight(), vfColorFrame_.getWidth(), CV_8UC3,
                              const_cast<void*>(vfColorFrame_.getData()));
      // Check if grabbed frame is actually full with some content
      cv::flip(mImageRGB, mImageRGB, 1);
      if (!mImageRGB.empty())
      {
        // Convert the cv image in a ROSy format
        msg_ = cv_bridge::CvImage(std_msgs::Header(), "rgb8", mImageRGB).toImageMsg();
        msg_->header.frame_id = rgb_frame;
        msg_->header.stamp = ros::Time::now();
        imagePub_.publish(msg_);
        // Publish the rgb camera info
        rgbInfoPub_.publish(this->fillCameraInfo(msg_->header.stamp, true));
        // cv::flip(mImageRGB, mImageRGB, 1);
        mImageRGB.copyTo(mImage); //uncomment
      }
      else
      {
        ROS_ERROR("Unable to get RGB video");
      }
      vfColorFrame_.release();
    }
    else
    {
      ROS_ERROR("Unable to get RGB video");
    }
  }
  /**
   * This method publishes the depth image on ROS
   */
  void getDepth()
  {
    // depthStream_.start();
    depthStream_.readFrame(&depthFrame_);
    // If the obtained frame is valid, then publish it over  ROS
    if (depthFrame_.isValid())
    {
      openni::DepthPixel* pData = (openni::DepthPixel*)depthFrame_.getData();
      cv::Mat image = cv::Mat(depthStream_.getVideoMode().getResolutionY(),
                              depthStream_.getVideoMode().getResolutionX(),
                              CV_16UC1,
                              pData);

      msg_depth = cv_bridge::CvImage(std_msgs::Header(), "16UC1", image).toImageMsg();
      msg_depth->header.frame_id = depth_frame;
      msg_depth->header.stamp = ros::Time::now();
      depthPub_.publish(msg_depth);
      // rgbInfoPub_.publish(this->fillCameraInfo(msg_->header.stamp, true));
      depthInfoPub_.publish(this->fillCameraInfo(msg_depth->header.stamp, false));
    }
    else
    {
      ROS_ERROR("Unable to publish depth-image");
    }
    // depthFrame_.release();

  }

  /**
   * Update the Users State
   * @param user: the user
   * @param ts: timestamp
   */
  void updateUserState(const nite::UserData& user, unsigned long long ts)
  {

    ::skeleton_tracker::skeleton_tracker_state state_msg;

    state_msg.userID = int(user.getId());
    state_msg.timepoint = ts;
    state_msg.message = "";
    state_msg.uuid = "";

    if (user.isNew()){
      USER_MESSAGE("New")
      state_msg.message = "New";
      now_str = num_to_str<double>(ros::Time::now().toSec());
      std::string uuid = generateUUID(now_str, state_msg.userID);
      std::cout << int(user.getId()) << uuid << std::endl;
      alluuid[int(user.getId())] = uuid;
      std::cout << "uuid" << int(user.getId()) <<": " << uuid << std::endl;
    }
    else if (user.isVisible() && !g_visibleUsers_[user.getId()]){
      USER_MESSAGE("Visible")
      state_msg.message = "Visible";
    }
    else if (!user.isVisible() && g_visibleUsers_[user.getId()]){
      USER_MESSAGE("Out of Scene")
      state_msg.message = "Out of Scene";
    }
    else if (user.isLost()){
      USER_MESSAGE("Lost")
      state_msg.message = "Lost";
    }

    state_msg.uuid = alluuid[int(user.getId())];

    g_visibleUsers_[user.getId()] = user.isVisible();

    if (g_skeletonStates_[user.getId()] != user.getSkeleton().getState())
    {
      switch (g_skeletonStates_[user.getId()] = user.getSkeleton().getState())
      {
        case nite::SKELETON_NONE:
          USER_MESSAGE("Stopped tracking")
          state_msg.message = "Stopped tracking";
          break;
        case nite::SKELETON_CALIBRATING:
          USER_MESSAGE("Calibrating...")
          state_msg.message = "Calibrating";
          break;
        case nite::SKELETON_TRACKED:
          USER_MESSAGE("Tracking!")
          state_msg.message = "Tracking";
          break;
        case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
        case nite::SKELETON_CALIBRATION_ERROR_HANDS:
        case nite::SKELETON_CALIBRATION_ERROR_LEGS:
        case nite::SKELETON_CALIBRATION_ERROR_HEAD:
        case nite::SKELETON_CALIBRATION_ERROR_TORSO:
          USER_MESSAGE("Calibration Failed... :-|")
          break;
      }
    }
    // Publish the state of the skeleton detection if it changes.
    if (state_msg.message != ""){
        skeleton_state_pub_.publish(state_msg);
        ros::Rate loop_rate(10);
    }
  }

  /**
   * Publish the joints over the TF stream
   * @param j_name: joint name
   * @param j: the joint
   * @param uid: user's ID
   */
  void publishJointTF(std::string j_name, nite::SkeletonJoint j, int uid)
  {

    if (j.getPositionConfidence() > 0.0)
    {
      tf::Transform transform;
      transform.setOrigin(
          tf::Vector3(j.getPosition().x / 1000.0, j.getPosition().y / -1000.0, j.getPosition().z / 1000.0));
      transform.setRotation(tf::Quaternion(0, 0, 0, 1));
      std::stringstream frame_id_stream;
      std::string frame_id;
      frame_id_stream << depth_frame << "/user_" << uid << "/" << j_name;
      frame_id = frame_id_stream.str();
      tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), depth_frame, frame_id));
    }
    return;
  }

  /**
   * Get the skeleton's joints and the users IDs
   */

  void getSkeleton()
  {
    int R1 = 31;
    int G1 = 210;
    int B1 = 170;
    skeleton_tracker::user_IDs ids;
    niteRc_ = userTracker_.readFrame(&userTrackerFrame_);
    if (niteRc_ != nite::STATUS_OK)
    {
      printf("Get next frame failed\n");
      return;
    }

    // Get all the users
    const nite::Array<nite::UserData>& users = userTrackerFrame_.getUsers();

    // Get the skeleton for every user
    for (int i = 0; i < users.getSize(); ++i)
    {
      const nite::UserData& user = users[i];
      updateUserState(user, userTrackerFrame_.getTimestamp());
      if (user.isNew())
      {
        userTracker_.startSkeletonTracking(user.getId());
      }
      else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
      {
        JointMap named_j;



/////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////

        incremental_msg.userID = int(user.getId());
        // if (int(user.getId())==1)
        incremental_msg.uuid = alluuid[int(user.getId())];
        incremental_msg.joints.clear();


        incremental_msg.joints.push_back(generate_joint_message("head",nite::JOINT_HEAD,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("neck",nite::JOINT_NECK,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("left_shoulder",nite::JOINT_LEFT_SHOULDER,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("right_shoulder",nite::JOINT_RIGHT_SHOULDER,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("left_elbow",nite::JOINT_LEFT_ELBOW,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("right_elbow",nite::JOINT_RIGHT_ELBOW,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("left_hand",nite::JOINT_LEFT_HAND,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("right_hand",nite::JOINT_RIGHT_HAND,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("torso",nite::JOINT_TORSO,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("left_hip",nite::JOINT_LEFT_HIP,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("right_hip",nite::JOINT_RIGHT_HIP,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("left_knee",nite::JOINT_LEFT_KNEE,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("right_knee",nite::JOINT_RIGHT_KNEE,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("left_foot",nite::JOINT_LEFT_FOOT,named_j, p, user));
        incremental_msg.joints.push_back(generate_joint_message("right_foot",nite::JOINT_RIGHT_FOOT,named_j, p, user));


        incremental_msg.time = ros::Time::now();
        incremental_msg_pub_.publish(incremental_msg);




        if (!mImage.empty())
        {
          mImage.copyTo(canvas); //uncomment
          mImage *= .4;
          canvas *= .6;
          int X[15];
          int Y[15];
        //   int joint_mappinga1[]={0,1,1,1,8,8};
        //   int joint_mappinga2[]={1,2,5,8,9,12};
          int joint_mapping1[]={2,3,5,6,9,10,12,13};
          int joint_mapping2[]={3,4,6,7,10,11,13,14};
          int R[]={0,0,0,0,170,255,255,255,170};
          int G[]={0,170,255,255,255,170,0,0,0};
          int B[]={255,255,170,0,0,0,0,170,255};
          generate_joint_coordinates("head",0,X, Y, named_j);
          generate_joint_coordinates("neck",1,X, Y, named_j);
          generate_joint_coordinates("torso",8,X, Y, named_j);
          generate_joint_coordinates("left_shoulder",2,X, Y, named_j);
          generate_joint_coordinates("left_elbow",3,X, Y, named_j);
          generate_joint_coordinates("left_hand",4,X, Y, named_j);
          generate_joint_coordinates("right_shoulder",5,X, Y, named_j);
          generate_joint_coordinates("right_elbow",6,X, Y, named_j);
          generate_joint_coordinates("right_hand",7,X, Y, named_j);
          generate_joint_coordinates("left_hip",9,X, Y, named_j);
          generate_joint_coordinates("left_knee",10,X, Y, named_j);
          generate_joint_coordinates("left_foot",11,X, Y, named_j);
          generate_joint_coordinates("right_hip",12,X, Y, named_j);
          generate_joint_coordinates("right_knee",13,X, Y, named_j);
          generate_joint_coordinates("right_foot",14,X, Y, named_j);

        //   for(int i=0;i<sizeof(joint_mappinga1)/sizeof(*joint_mappinga1);i++)
            // cv::line(canvas, cv::Point(X[joint_mappinga1[i]],Y[joint_mappinga1[i]]),cv::Point(X[joint_mapping2[i]],Y[joint_mapping2[i]]),cv::Scalar(R1,G1,B1),3);

          for(int i=0;i<sizeof(joint_mapping1)/sizeof(*joint_mapping1);i++)
            {
                int x1 = Y[joint_mapping1[i]];
                int x2 = Y[joint_mapping2[i]];
                int y1 = X[joint_mapping1[i]];
                int y2 = X[joint_mapping2[i]];
                int mX = (x1+x2)/2;
                int mY = (y1+y2)/2;
                int length = std::pow(std::pow(x1-x2,2) + std::pow(y1-y2,2),.5)/2;
                int angle = std::atan2(x1 - x2, y1 - y2)*180/3.14;
                cv::ellipse( canvas, cv::Point(mY,mX), cv::Size(length, 9), angle, 0, 360, cv::Scalar( R[i], G[i], B[i] ), -1, 8);
            }

          cv::circle(canvas, cv::Point(X[0],Y[0]), 8.0, cv::Scalar(220,20,240), -1, 1 );
          mImage += canvas;
        }

        for (JointMap::iterator it = named_j.begin(); it != named_j.end(); ++it)
        {
          publishJointTF(it->first, it->second, user.getId());
        }
        // Add the user's ID
        ids.users.push_back(int(user.getId()));
      }
    }
    // Publish the users' IDs
    userPub_.publish(ids);
    msg_ = cv_bridge::CvImage(std_msgs::Header(), "rgb8", mImage).toImageMsg();
    msg_->header.frame_id = depth_frame;
    msg_->header.stamp = ros::Time::now();
    imageSKPub_.publish(msg_);
  }

  /**
   * Method that returns information about the camera
   * @param time: ros timestamp
   * @param is_rgb
   * @return the \ref sensor_msgs::CameraInfoPtr message
   */
  sensor_msgs::CameraInfoPtr fillCameraInfo(ros::Time time, bool is_rgb)
  {
    sensor_msgs::CameraInfoPtr info_msg = boost::make_shared<sensor_msgs::CameraInfo>();
    info_msg->header.frame_id = depth_frame;
    if (is_rgb)
    {
      info_msg->header.frame_id = rgb_frame;
    }
    info_msg->header.stamp = time;
    if (read_yaml_success)
    {
        info_msg->width = width;
        info_msg->height = height;
        // note to myself: fix distortion model
        info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        info_msg->D = D;
        for( int a = 0; a < 9; a = a + 1 ) {
            info_msg->K[a] = K[a];
            info_msg->R[a] = R[a];
            info_msg->P[a] = P[a];
        }
        info_msg->P[9] = P[9];
        info_msg->P[10] = P[10];
        info_msg->P[11] = P[11];
    }
    else
    {
        info_msg->width = is_rgb ? mMode_.getResolutionX() : depthMode_.getResolutionX();
        info_msg->height = is_rgb ? mMode_.getResolutionY() : depthMode_.getResolutionY();
        info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        info_msg->D = std::vector<double>(5, 0.0);
        info_msg->K.assign(0.0);
        info_msg->R.assign(0.0);
        info_msg->P.assign(0.0);
        openni::DeviceInfo info = devDevice_.getDeviceInfo();
        const char* uri = info.getUri();
        std::string stringa(uri);
        openni2_wrapper::OpenNI2Device dev(stringa);

        double f =
            is_rgb ? dev.getColorFocalLength(vfColorFrame_.getHeight()) : dev.getColorFocalLength(depthFrame_.getHeight());
        info_msg->K[0] = info_msg->K[4] = f;
        info_msg->K[2] = (info_msg->width / 2) - 0.5;
        info_msg->K[5] = (info_msg->width * 3. / 8.) - 0.5; //aspect ratio for the camera center on kinect and presumably other devices is 4/3
        info_msg->K[8] = 1.0;
        // no rotation: identity
        info_msg->R[0] = info_msg->R[4] = info_msg->R[8] = 1.0;
        // no rotation, no translation => P=K(I|0)=(K|0)
        info_msg->P[0] = info_msg->P[5] = info_msg->K[0];
        info_msg->P[2] = info_msg->K[2];
        info_msg->P[6] = info_msg->K[5];
        info_msg->P[10] = 1.0;
    }

    return (info_msg);
  }

  /// ROS NodeHandle
  ros::NodeHandle nh_;

  bool g_visibleUsers_[MAX_USERS] = {false};
  nite::SkeletonState g_skeletonStates_[MAX_USERS] = {nite::SKELETON_NONE};

  /// Image transport
  image_transport::ImageTransport it_;
  std::string depth_frame, rgb_frame, camera, camera_calibration;
  /// Frame broadcaster
  tf::TransformBroadcaster tfBroadcast_;
  /// The openni device
  openni::Device devDevice_;
  /// Some NITE stuff
  nite::UserTracker userTracker_;
  nite::Status niteRc_;
  nite::UserTrackerFrameRef userTrackerFrame_;
  /// Video color stream
  openni::VideoStream vsColorStream_;
  /// OpenNI video mode
  openni::VideoMode mMode_;
  /// OpenNI depth frame reference
  openni::VideoFrameRef depthFrame_;
  /// OpenNI depth stream
  openni::VideoStream depthStream_;
  /// OpenNI depth mode
  openni::VideoMode depthMode_;
  /// OpenNI video frame reference
  openni::VideoFrameRef vfColorFrame_;
  /// RGB image publisher
  image_transport::Publisher imagePub_;
  /// RGB sk tracks image publisher
  image_transport::Publisher imageSKPub_;


  // image_transport::Publisher imageWhitePub_;
  cv::Mat mImage;
  cv::Mat canvas;
  // cv::Mat mImage_white;
  /// Users IDs publisher
  ros::Publisher userPub_;
  /// Point cloud publisher
  ros::Publisher pointCloudPub_;

  ros::Subscriber sub;
  /// Image message
  sensor_msgs::ImagePtr msg_;

  sensor_msgs::ImagePtr msg_depth;
  /// Node rate
  ros::Rate* rate_;
  /// Depth image publisher
  image_transport::Publisher depthPub_;

  ///RGB Camera INFO
  sensor_msgs::CameraInfoPtr rgbInfo_;
  ///RGB Camera info publisher
  ros::Publisher rgbInfoPub_;

  /// Depth Info
  sensor_msgs::CameraInfoPtr depthInfo_;
  /// Depth info publisher
  ros::Publisher depthInfoPub_;

  /// variable to save the uuids for users
  std::string alluuid[100];

  //camera_calibrations
  int width=0;
  int height=0;
  std::vector<double> K;
  std::vector<double> R;
  std::vector<double> D;
  std::vector<double> P;

  bool read_yaml_success=false;

  ::geometry_msgs::Pose p;
  std::string joint_name;

  ::skeleton_tracker::skeleton_message incremental_msg;             // left foot
  ros::Publisher incremental_msg_pub_;

  // State of skeleton tracker publisher
  ros::Publisher skeleton_state_pub_;

  boost::uuids::uuid dns_namespace_uuid;
  std::string now_str = num_to_str<double>(ros::Time::now().toSec());
};

#endif /* XTION_TRACKER_HPP_ */

