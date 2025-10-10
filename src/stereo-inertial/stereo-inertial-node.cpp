#include "stereo-inertial-node.hpp"

#include<opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <filesystem>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode() :
    Node("ORB_SLAM3_ROS2")
{
    this->declare_parameter<std::string>("config", "");
    this->declare_parameter<std::string>("vocabulary", "");
    this->declare_parameter<std::string>("output_folder", "");
    this->declare_parameter<bool>("m_rectify", false);
    this->declare_parameter<bool>("m_equal", false);
    this->declare_parameter<bool>("visualization", false);

    // Get parameter values
    std::string config_file = this->get_parameter("config").as_string();
    std::string vocabulary_file = this->get_parameter("vocabulary").as_string();
    m_output_folder = this->get_parameter("output_folder").as_string();
    bool m_rectify = this->get_parameter("m_rectify").as_bool();
    bool m_equal = this->get_parameter("m_equal").as_bool();
    bool visualization = this->get_parameter("visualization").as_bool();

    bClahe_ = m_equal;

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

    SLAM_ = std::make_unique<ORB_SLAM3::System>(vocabulary_file, config_file, ORB_SLAM3::System::IMU_STEREO, visualization);

    if (m_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
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

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "camera_pose", 10);

    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("image_left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("image_right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));

    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();
}

void StereoInertialNode::saveMapOnShutdown()
{
    // Create output folder if it doesn't exist
    std::string output_folder = m_output_folder;
    if (!std::filesystem::exists(output_folder))
    {
        std::filesystem::create_directories(output_folder);
    }

    // // Save camera trajectory
    RCLCPP_INFO(this->get_logger(), "Saving camera trajectory to %s", (m_output_folder + "/trajectory.txt").c_str());
    SLAM_->SaveKeyFrameTrajectoryTUM(m_output_folder + "/trajectory.txt");

    // Get all map points
    ORB_SLAM3::Atlas* atlas = nullptr;
    atlas = SLAM_->GetAtlas();

    RCLCPP_INFO(this->get_logger(), "Saving Atlas file");
    SLAM_->SaveAtlas(FileType::BINARY_FILE);

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
    RCLCPP_INFO(this->get_logger(), "Done Saving Map");
}



void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                RCLCPP_DEBUG(this->get_logger(), "Too big time difference between left and right images: %f", tImLeft - tImRight);
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            Sophus::SE3f Tcw;
            Tcw = SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

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
                pose_msg.header.stamp = imuBuf_.back()->header.stamp;
                pose_msg.header.frame_id = imuBuf_.back()->header.frame_id;

                pose_msg.pose.position.x = twc(0);
                pose_msg.pose.position.y = twc(1);
                pose_msg.pose.position.z = twc(2);
                pose_msg.pose.orientation = tf2::toMsg(q);

                pose_pub_->publish(pose_msg);
            }

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
