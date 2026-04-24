#include <gtest/gtest.h>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_vision/librairies/yolo_detector.hpp"

using namespace robot_vision;

TEST(YoloDetectorTest, TestProviderName)
{
    std::string_view providerName("CPU");

    YoloDetector detector(ament_index_cpp::get_package_share_directory("robot_vision") + "/models/yolov8n.onnx", false);
    EXPECT_EQ(detector.providerName(), providerName);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}