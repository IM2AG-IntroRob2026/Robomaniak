#include <gtest/gtest.h>
#include <string>
#include "robot_vision/librairies/yolo_detector.hpp"

using namespace robot_vision;

TEST(YoloDetectorTest, TestProviderName)
{
    std::string_view providerName("CPU");

    YoloDetector detector("yolov8s.onnx", false);
    EXPECT_EQ(detector.providerName(), providerName);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}