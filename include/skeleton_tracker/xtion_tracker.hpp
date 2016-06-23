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
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <skeleton_tracker/user_IDs.h>
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "skeleton_tracker/skeleton_tracker_state.h"
#include <iomanip>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>


#include <iostream>
#include <vector>

#ifndef ALPHA
#define ALPHA 1/256
#endif

#define MAX_USERS 2

#define USER_MESSAGE(msg) \
        {printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

/// Joint map
typedef std::map<std::string, nite::SkeletonJoint> JointMap;

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



// std::cout << R1[0] << std::endl;
/**
 * Class \ref xtion_tracker. This class can track the skeleton of people and returns joints as a TF stream,
 *  while publishing the video stream and the point cloud captured by an ASUS Xtion Pro Live.
 */
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
    // if (!pnh.getParam("tf_prefix", tf_prefix_))
    // {
    //   ROS_FATAL("tf_prefix not found on Param Server! Maybe you should add it to your launch file!");
    //   ros::shutdown();
    //   return;
    // }
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
    // imageWhitePub_ = it_.advertise("/"+camera+"/rgb/white_sk_tracks", 1);

    // Initialize the point cloud publisher
    // pointCloudPub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("/"+camera+"/depth_registered/points", 5);

    // Initialize the depth image publisher
    depthPub_ = it_.advertise("/"+camera+"/depth/image_raw", 1);

    // Initialize the users IDs publisher
    userPub_ = nh_.advertise<skeleton_tracker::user_IDs>("/people", 1);

    // Initialize both the Camera Info publishers
    depthInfoPub_ = nh_.advertise<sensor_msgs::CameraInfo>("/"+camera+"/depth/camera_info", 1);

    rgbInfoPub_ = nh_.advertise<sensor_msgs::CameraInfo>("/"+camera+"/rgb/camera_info", 1);

    // rate_ = new ros::Rate(300);

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
      state_msg.uuid = uuid;
     }

    else if (user.isVisible() && !g_visibleUsers_[user.getId()])
      USER_MESSAGE("Visible")
    else if (!user.isVisible() && g_visibleUsers_[user.getId()]){
      USER_MESSAGE("Out of Scene")
      state_msg.message = "Out of Scene";}
    else if (user.isLost())
      USER_MESSAGE("Lost")

    g_visibleUsers_[user.getId()] = user.isVisible();

    if (g_skeletonStates_[user.getId()] != user.getSkeleton().getState())
    {
      switch (g_skeletonStates_[user.getId()] = user.getSkeleton().getState())
      {
        case nite::SKELETON_NONE:
          USER_MESSAGE("Stopped tracking.")
          break;
        case nite::SKELETON_CALIBRATING:
          USER_MESSAGE("Calibrating...")
          break;
        case nite::SKELETON_TRACKED:
          USER_MESSAGE("Tracking!")
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
          tf::Vector3(j.getPosition().x / 1000.0, j.getPosition().y / 1000.0, j.getPosition().z / 1000.0));
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

    float fx = 525.0;
    float fy = 525.0;
    float cx = 319.5;
    float cy = 239.5;

    int R1 = 31;
    int G1 = 210;
    int B1 = 170;


    // std::vector<int> R1 (1);
    // std::vector<int> G1 (1);
    // std::vector<int> B1 (1);
    // R1[0] = 255;
    // R1[1] = 0;
    // R1[2] = 0;
    // R1[3] = 0;
    // R1[4] = 255;
    // R1[5] = 255;
    // R1[6] = 128;
    // R1[7] = 128;
    // R1[8] = 255;
    // R1[9] = 40;

    // G1[0] = 0;
    // G1[1] = 255;
    // G1[2] = 0;
    // G1[3] = 255;
    // G1[4] = 0;
    // G1[5] = 128;
    // G1[6] = 128;
    // G1[7] = 0;
    // G1[8] = 128;
    // G1[9] = 200;

    // B1[0] = 80;
    // B1[1] = 0;
    // B1[2] = 255;
    // B1[3] = 255;
    // B1[4] = 255;
    // B1[5] = 128;
    // B1[6] = 128;
    // B1[7] = 0;
    // B1[8] = 0;
    // B1[9] = 100;

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
        JointMap named_joints;

        named_joints["head"] = (user.getSkeleton().getJoint(nite::JOINT_HEAD));
        named_joints["neck"] = (user.getSkeleton().getJoint(nite::JOINT_NECK));
        named_joints["left_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER));
        named_joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER));
        named_joints["left_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW));
        named_joints["right_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW));
        named_joints["left_hand"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND));
        named_joints["right_hand"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND));
        named_joints["torso"] = (user.getSkeleton().getJoint(nite::JOINT_TORSO));
        named_joints["left_hip"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP));
        named_joints["right_hip"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP));
        named_joints["left_knee"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE));
        named_joints["right_knee"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE));
        named_joints["left_foot"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT));
        named_joints["right_foot"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT));

        if (!mImage.empty())
        {
          // int X = 0;
          // int Y = 0;
          int XH = int(named_joints["head"].getPosition().x*fx/named_joints["head"].getPosition().z +cx);
          int YH = int(named_joints["head"].getPosition().y*fy/named_joints["head"].getPosition().z*-1 +cy);
          int XN = int(named_joints["neck"].getPosition().x*fx/named_joints["neck"].getPosition().z*1 +cx);
          int YN = int(named_joints["neck"].getPosition().y*fy/named_joints["neck"].getPosition().z*-1 +cy);
          int XT = int(named_joints["torso"].getPosition().x*fx/named_joints["torso"].getPosition().z*1 +cx);
          int YT = int(named_joints["torso"].getPosition().y*fy/named_joints["torso"].getPosition().z*-1 +cy);

          int XLS = int(named_joints["left_shoulder"].getPosition().x*fx/named_joints["left_shoulder"].getPosition().z*1 +cx);
          int YLS = int(named_joints["left_shoulder"].getPosition().y*fy/named_joints["left_shoulder"].getPosition().z*-1 +cy);
          int XLE = int(named_joints["left_elbow"].getPosition().x*fx/named_joints["left_elbow"].getPosition().z*1 +cx);
          int YLE = int(named_joints["left_elbow"].getPosition().y*fy/named_joints["left_elbow"].getPosition().z*-1 +cy);
          int XLH = int(named_joints["left_hand"].getPosition().x*fx/named_joints["left_hand"].getPosition().z*1 +cx);
          int YLH = int(named_joints["left_hand"].getPosition().y*fy/named_joints["left_hand"].getPosition().z*-1 +cy);

          int XRS = int(named_joints["right_shoulder"].getPosition().x*fx/named_joints["right_shoulder"].getPosition().z*1 +cx);
          int YRS = int(named_joints["right_shoulder"].getPosition().y*fy/named_joints["right_shoulder"].getPosition().z*-1 +cy);
          int XRE = int(named_joints["right_elbow"].getPosition().x*fx/named_joints["right_elbow"].getPosition().z*1 +cx);
          int YRE = int(named_joints["right_elbow"].getPosition().y*fy/named_joints["right_elbow"].getPosition().z*-1 +cy);
          int XRH = int(named_joints["right_hand"].getPosition().x*fx/named_joints["right_hand"].getPosition().z*1 +cx);
          int YRH = int(named_joints["right_hand"].getPosition().y*fy/named_joints["right_hand"].getPosition().z*-1 +cy);

          int XRHip = int(named_joints["right_hip"].getPosition().x*fx/named_joints["right_hip"].getPosition().z*1 +cx);
          int YRHip = int(named_joints["right_hip"].getPosition().y*fy/named_joints["right_hip"].getPosition().z*-1 +cy);
          int XRK = int(named_joints["right_knee"].getPosition().x*fx/named_joints["right_knee"].getPosition().z*1 +cx);
          int YRK = int(named_joints["right_knee"].getPosition().y*fy/named_joints["right_knee"].getPosition().z*-1 +cy);
          int XRF = int(named_joints["right_foot"].getPosition().x*fx/named_joints["right_foot"].getPosition().z*1 +cx);
          int YRF = int(named_joints["right_foot"].getPosition().y*fy/named_joints["right_foot"].getPosition().z*-1 +cy);

          int XLHip = int(named_joints["left_hip"].getPosition().x*fx/named_joints["left_hip"].getPosition().z*1 +cx);
          int YLHip = int(named_joints["left_hip"].getPosition().y*fy/named_joints["left_hip"].getPosition().z*-1 +cy);
          int XLK = int(named_joints["left_knee"].getPosition().x*fx/named_joints["left_knee"].getPosition().z*1 +cx);
          int YLK = int(named_joints["left_knee"].getPosition().y*fy/named_joints["left_knee"].getPosition().z*-1 +cy);
          int XLF = int(named_joints["left_foot"].getPosition().x*fx/named_joints["left_foot"].getPosition().z*1 +cx);
          int YLF = int(named_joints["left_foot"].getPosition().y*fy/named_joints["left_foot"].getPosition().z*-1 +cy);

          cv::line(mImage, cv::Point(XH,YH),cv::Point(XN,YN),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XN,YN),cv::Point(XT,YT),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XN,YN),cv::Point(XLS,YLS),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XLS,YLS),cv::Point(XLE,YLE),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XLE,YLE),cv::Point(XLH,YLH),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XN,YN),cv::Point(XRS,YRS),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XRS,YRS),cv::Point(XRE,YRE),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XRE,YRE),cv::Point(XRH,YRH),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XT,YT),cv::Point(XRHip,YRHip),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XT,YT),cv::Point(XLHip,YLHip),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XRHip,YRHip),cv::Point(XRK,YRK),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XLHip,YLHip),cv::Point(XLK,YLK),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XRK,YRK),cv::Point(XRF,YRF),cv::Scalar(R1,G1,B1),3);
          cv::line(mImage, cv::Point(XLK,YLK),cv::Point(XLF,YLF),cv::Scalar(R1,G1,B1),3);
          cv::circle(mImage, cv::Point(XH,YH), 8.0, cv::Scalar(R1,G1,B1), -1, 1 );

          // cv::line(mImage, cv::Point(XH,YH),cv::Point(XN,YN),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XN,YN),cv::Point(XT,YT),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XN,YN),cv::Point(XLS,YLS),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XLS,YLS),cv::Point(XLE,YLE),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XLE,YLE),cv::Point(XLH,YLH),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XN,YN),cv::Point(XRS,YRS),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XRS,YRS),cv::Point(XRE,YRE),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XRE,YRE),cv::Point(XRH,YRH),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XT,YT),cv::Point(XRHip,YRHip),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XT,YT),cv::Point(XLHip,YLHip),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XRHip,YRHip),cv::Point(XRK,YRK),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XLHip,YLHip),cv::Point(XLK,YLK),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XRK,YRK),cv::Point(XRF,YRF),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::line(mImage, cv::Point(XLK,YLK),cv::Point(XLF,YLF),cv::Scalar(R1[i],G1[i],B1[i]),3);
          // cv::circle(mImage, cv::Point(XH,YH), 8.0, cv::Scalar(R1[i],G1[i],B1[i]), -1, 1 );
        }

        for (JointMap::iterator it = named_joints.begin(); it != named_joints.end(); ++it)
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
    // if (!is_rgb)
    // {
    //   depthStream_.start();
    //   depthStream_.readFrame(&depthFrame_);
    // }

    if (is_rgb)
    {
      info_msg->header.frame_id = rgb_frame;
    }

    info_msg->header.frame_id = depth_frame;
    info_msg->header.stamp = time;
    info_msg->width = is_rgb ? mMode_.getResolutionX() : depthMode_.getResolutionX();
    info_msg->height = is_rgb ? mMode_.getResolutionY() : depthMode_.getResolutionY();
    info_msg->D = std::vector<double>(5, 0.0);
    info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
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
    return (info_msg);
  }


  /// ROS NodeHandle
  ros::NodeHandle nh_;

  bool g_visibleUsers_[MAX_USERS] = {false};
  nite::SkeletonState g_skeletonStates_[MAX_USERS] = {nite::SKELETON_NONE};

  /// Image transport
  image_transport::ImageTransport it_;
  std::string depth_frame, rgb_frame, camera;
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
  // cv::Mat mImage_white;
  /// Users IDs publisher
  ros::Publisher userPub_;
  /// Point cloud publisher
  ros::Publisher pointCloudPub_;
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


  // State of skeleton tracker publisher
  ros::Publisher skeleton_state_pub_;

  boost::uuids::uuid dns_namespace_uuid;
  std::string now_str = num_to_str<double>(ros::Time::now().toSec());
}
;

#endif /* XTION_TRACKER_HPP_ */
