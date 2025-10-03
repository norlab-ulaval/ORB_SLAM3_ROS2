#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    // m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ORB_SLAM3::Atlas* atlas = nullptr;

    // In ORB-SLAM3 ROS2 wrapper, mpAtlas may not be public.
    // If you have access via m_SLAM->GetAtlas(), use that.
    // Otherwise, if only private/protected, you can only iterate
    // KeyFrames / MapPoints that are accessible via System methods.
    // Here we assume your wrapper can access atlas pointer:
    atlas = m_SLAM->GetAtlas(); // If not, use friend class / wrapper in your repo.

    if (!atlas) {
        std::cerr << "Atlas pointer is null, cannot dump map" << std::endl;
        return;
    }

    std::ofstream f("map.txt");
    if (!f.is_open()) {
        std::cerr << "Cannot open MapDump.txt for writing" << std::endl;
        return;
    }

    // Dump KeyFrames
    f << "# KeyFrames\n";
    for (auto* kf : atlas->GetAllKeyFrames()) {
        if (!kf || kf->isBad()) continue;

        Sophus::SE3f pose = kf->GetPoseInverse();
        Eigen::Quaternionf q = pose.unit_quaternion();
        Eigen::Vector3f t = pose.translation();

        f << "KF " << kf->mnId << " " << kf->mTimeStamp << " "
          << t.x() << " " << t.y() << " " << t.z() << " "
          << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }

    // Dump MapPoints
    f << "# MapPoints\n";
    for (auto* mp : atlas->GetAllMapPoints()) {
        if (!mp || mp->isBad()) continue;

        Eigen::Vector3f pos = mp->GetWorldPos();
        f << "MP " << mp->mnId << " "
          << pos.x() << " "
          << pos.y() << " "
          << pos.z() << "\n";
    }

    f.close();
    std::cout << "Saved full map to map.txt" << std::endl;
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
}
