#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>

#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>

namespace robot_vision
{

// YOLOv8
inline constexpr int   YOLO_INPUT_SIZE       = 640;
inline constexpr int   YOLO_NUM_CLASSES      = 80;
inline constexpr int   YOLO_NUM_ANCHORS      = 8400; // for 640x640
inline constexpr int   YOLO_CLASS_PERSON     = 0;
inline constexpr float YOLO_DEFAULT_CONF     = 0.50f;
inline constexpr float YOLO_DEFAULT_NMS_IOU  = 0.45f;
inline constexpr uint8_t LETTERBOX_FILL      = 114;

struct Detection
{
    float cx{}; // center X (in original image space)
    float cy{}; // center Y
    float w{};
    float h{};
    float confidence{};
    int class_id{};
};

struct LetterboxParams
{
    float scale{1.0f};
    float pad_x{0.0f};
    float pad_y{0.0f};
};

enum class GpuProvider { None, CUDA, ROCm };

class YoloDetector
{
private :
    // Attributs
    float conf_thresh_{YOLO_DEFAULT_CONF};
    float nms_iou_{YOLO_DEFAULT_NMS_IOU};
    GpuProvider gpu_provider_{GpuProvider::None};
    Ort::Env ort_env_;
    std::unique_ptr<Ort::Session> session_;

public:
    // Constructor
    /**
     * Initialise an ONNX Runtime session with the specified YOLOv8 model.
     * If use_gpu is true, it will attempt to use CUDA (Nvidia) or ROCm (AMD) if available;
     * otherwise, it will fall back to CPU execution.
     * The confidence threshold is used during post-processing to filter out low-confidence detections.
     * Throws a runtime_error if the model file is not found.
     *
     * @param model_path Path to the ONNX model file (e.g., yolov8s.onnx).
     * @param use_gpu Whether to attempt using GPU acceleration (CUDA/ROCm).
     * @param conf_thresh Minimum confidence threshold for keeping detections (default: 0.50).
	 * @param nms_iou IoU threshold for Non-Maximum Suppression (default: 0.45).
     */
    explicit YoloDetector(const std::filesystem::path& model_path, bool use_gpu = true, float conf_thresh = YOLO_DEFAULT_CONF, float nms_iou = YOLO_DEFAULT_NMS_IOU);
    
    // Methods
    [[nodiscard]] GpuProvider gpuProvider()  const noexcept;
    [[nodiscard]] bool gpuEnabled() const noexcept;

    [[nodiscard]] std::string_view providerName() const noexcept;

    /**
     * Run inference on the given BGR frame and return a vector of detections.
     * The input frame is preprocessed (resized with letterbox, color converted, normalized)
     * before being fed to the model, and the raw output is post-processed to extract bounding boxes,
     * confidence scores, and class IDs.
     *
     * @param bgr_frame Input image in BGR format (as typically used by OpenCV).
     * @return A vector of Detection objects representing the detected objects in the original image space.
     */
    [[nodiscard]] std::vector<Detection> detect(const cv::Mat& bgr_frame);

private:
    /**
     * Run the preprocessing steps required by YOLOv8:
     * 1. Compute the scaling factor to fit the input image into a 640x640 box while preserving aspect ratio.
     * 2. Resize the image using OpenCV's INTER_LINEAR interpolation.
     * 3. Add letterbox padding (filled with a constant value) to reach the final 640x640 size.
     * 4. Convert color space from BGR to RGB.
     * 5. Normalize pixel values to the [0,1] range and rearrange the data from HWC (height, width, channels) to CHW (channels, height, width) format expected by the model.
     * The function also fills the LetterboxParams struct with the scaling and padding information, which is needed later for post-processing the detections back into the original image space.
     *
     * @param src Input image in BGR format.
     * @param lb Output parameter to receive the letterbox scaling and padding information.
     * @return A vector of floats representing the preprocessed image tensor in CHW format,
     */
    [[nodiscard]] std::vector<float> preprocess(const cv::Mat& src, LetterboxParams& lb) const;

    /**
     * Run inference on the preprocessed input tensor using the ONNX Runtime session.
     * The function prepares the input tensor in the format expected by the model, executes the session, and retrieves the raw output tensor as a vector of floats.
     * The output is expected to be in the shape [1, 84, 8400], where 84 corresponds to the 4 bounding box parameters (x_center, y_center, width, height)
     * plus 80 class confidence scores, and 8400 corresponds to the number of anchor points for a 640x640 input.
     *
     * @param input A vector of floats representing the preprocessed image tensor in CHW format.
     * @return A vector of floats containing the raw output from the model, which will be post-processed to extract detections.
     */
    [[nodiscard]] std::vector<float> runInference(const std::vector<float>& input) const;

    /**
     * Post-process the raw output from the model to extract detections.
     * The function iterates over each of the 8400 anchor points,
     * checks if the confidence score for the "person" class exceeds the specified threshold,
     * and if so, it constructs a Detection object with the bounding box parameters (center coordinates, width, height)
     * adjusted back to the original image space using the letterbox parameters.
     * The resulting vector of Detection objects contains only those detections
     * that are classified as "person" with a confidence above the threshold.
     *
     * @param output A vector of floats containing the raw output from the model, expected to be in the shape [1, 84, 8400].
     * @param lb The LetterboxParams struct containing the scaling and padding information from the preprocessing step, used to convert the bounding box coordinates back to the original image space.
     * @return A vector of Detection objects representing the detected "person" instances in the original image space, filtered by the confidence threshold.
     */
    [[nodiscard]] std::vector<Detection> postprocess(const std::vector<float>& output, const LetterboxParams& lb) const;

	/**
	 * Compute the Intersection over Union (IoU) between two detections.
     * The function calculates the coordinates of the intersection rectangle between the two bounding boxes,
     * computes the area of the intersection, and then divides it by the area of the union of the two boxes.
     * The IoU value ranges from 0 to 1, where 0 means no overlap and 1 means perfect overlap.
     * This metric is used during Non-Maximum Suppression (NMS) to determine whether two detections are sufficiently overlapping to be considered duplicates.
     *
     * @param a The first Detection object.
     * @param b The second Detection object.
     * @return A float value representing the IoU between the two detections.
     */
    [[nodiscard]] static float iou(const Detection& a, const Detection& b) noexcept;

    /**
     * Apply Non-Maximum Suppression (NMS) to a list of candidate detections to filter out duplicates.
     * The function first sorts the detections by confidence score in descending order,
     * then iteratively selects the highest-confidence detection and suppresses any subsequent detections that have an IoU greater than the specified threshold with the selected detection.
     * The result is a vector of Detection objects that are considered valid and non-overlapping according to the NMS criteria.
     *
     * @param dets A vector of Detection objects representing candidate detections before NMS.
     * @return A vector of Detection objects that have been filtered by NMS, containing only the most confident detections without significant overlap.
     */
    [[nodiscard]] std::vector<Detection> applyNMS(std::vector<Detection> dets) const;
}; // class YoloDetector

} // namespace robot_vision