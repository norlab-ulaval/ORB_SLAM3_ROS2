#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode()
:   Node("ORB_SLAM3_ROS2")
{

    this->declare_parameter<std::string>("config", "");
    this->declare_parameter<std::string>("vocabulary", "");
    this->declare_parameter<std::string>("output_folder", "");
    this->declare_parameter<bool>("m_rectify", false);
    this->declare_parameter<bool>("visualization", false);

    // Get parameter values
    std::string config_file = this->get_parameter("config").as_string();
    std::string vocabulary_file = this->get_parameter("vocabulary").as_string();
    m_output_folder = this->get_parameter("output_folder").as_string();
    bool m_rectify = this->get_parameter("m_rectify").as_bool();
    bool visualization = this->get_parameter("visualization").as_bool();

    if (config_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "config parameter is required but not provided");
            rclcpp::shutdown();
            return;
        }

    if (vocabulary_file.empty()) {
        RCLCPP_ERROR(this->get_logger(), "vocabulary parameter is required but not provided");
        rclcpp::shutdown();
        return;
    }

    m_SLAM = std::make_unique<ORB_SLAM3::System>(vocabulary_file, config_file, ORB_SLAM3::System::STEREO, visualization);

    if (m_rectify){

        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to m_rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "camera_pose", 10);


    left_sub = std::make_shared< message_filters::Subscriber<ImageMsg> >(this, "image_left");
    right_sub = std::make_shared< message_filters::Subscriber<ImageMsg> >(this, "image_right");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();
}

void StereoSlamNode::saveMapOnShutdown()
{
    // Create output folder if it doesn't exist
    std::string output_folder = m_output_folder;
    if (!std::filesystem::exists(output_folder))
    {
        std::filesystem::create_directories(output_folder);
    }

    // Save camera trajectory
    RCLCPP_INFO(this->get_logger(), "Saving camera trajectory to %s", (m_output_folder + "/trajectory.txt").c_str());
    m_SLAM->SaveKeyFrameTrajectoryTUM(m_output_folder + "/trajectory.txt");

    // Get all map points
    ORB_SLAM3::Atlas* atlas = nullptr;
    atlas = m_SLAM->GetAtlas();

    // Save to file
    RCLCPP_INFO(this->get_logger(), "Saving point cloud to %s", (m_output_folder + "/pointcloud.csv").c_str());
    std::string filename = m_output_folder + "/pointcloud.csv";
    std::ofstream file(filename);
    for(ORB_SLAM3::MapPoint* pMP : atlas->GetAllMapPoints())
    {
        if(pMP && !pMP->isBad())
        {
            Eigen::Vector3f pos = pMP->GetWorldPos();
            file << pos.x() << "," << pos.y() << "," << pos.z() << std::endl;
        }
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "Done");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw;

    if (m_rectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    }
    else
    {
        Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
    }

    if (Tcw.translation().norm() != 0)  // use translation norm as a simple check
    {
        Eigen::Matrix3f Rwc = Tcw.rotationMatrix().transpose();  // world <- camera
        Eigen::Vector3f twc = -Rwc * Tcw.translation();

        tf2::Matrix3x3 tf2_R(
            Rwc(0,0), Rwc(0,1), Rwc(0,2),
            Rwc(1,0), Rwc(1,1), Rwc(1,2),
            Rwc(2,0), Rwc(2,1), Rwc(2,2)
        );

        tf2::Quaternion q;
        tf2_R.getRotation(q);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msgLeft->header.stamp;
        pose_msg.header.frame_id = msgLeft->header.frame_id;

        pose_msg.pose.position.x = twc(0);
        pose_msg.pose.position.y = twc(1);
        pose_msg.pose.position.z = twc(2);
        pose_msg.pose.orientation = tf2::toMsg(q);

        pose_pub_->publish(pose_msg);
    }
}
