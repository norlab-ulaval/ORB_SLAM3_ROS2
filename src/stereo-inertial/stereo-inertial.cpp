#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    auto node = std::make_shared<StereoInertialNode>();

    // Register shutdown callback on the global context
    auto context = rclcpp::contexts::get_global_default_context();

    // Use a weak pointer to avoid keeping the node alive
    std::weak_ptr<StereoInertialNode> weak_node = node;

    context->add_on_shutdown_callback(
        [weak_node]() {
            if (auto n = weak_node.lock()) {
                RCLCPP_INFO(n->get_logger(), "Received a shut down call");
                n->saveMapOnShutdown();
            }
        });

    try {
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
