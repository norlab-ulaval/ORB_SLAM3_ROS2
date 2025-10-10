#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode();
    ~StereoInertialNode();
    void saveMapOnShutdown();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);
    void GrabImageRight(const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;

    std::unique_ptr<ORB_SLAM3::System> SLAM_;
    std::thread *syncThread_;

    std::string m_output_folder;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<ImageMsg::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    bool doRectify_;
    bool doEqual_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    // Thread control
    std::atomic<bool> shutdown_requested_;
};

#endif
