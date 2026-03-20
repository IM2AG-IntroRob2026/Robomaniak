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
public:
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
    explicit YoloDetector(const std::filesystem::path& model_path, bool use_gpu = true, float conf_thresh = YOLO_DEFAULT_CONF, float nms_iou = YOLO_DEFAULT_NMS_IOU)
        : conf_thresh_(conf_thresh), nms_iou_(nms_iou), ort_env_(ORT_LOGGING_LEVEL_WARNING, "yolo_detector")
    {
        if (!std::filesystem::exists(model_path)) {
            throw std::runtime_error("Model ONNX not found : " + model_path.string());
        }

        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(4);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        if (use_gpu) {
            // First, we try to append the CUDA execution provider. If it fails (e.g., due to missing CUDA libraries), we catch the exception and try ROCm next.
            // Needs onnxruntime built with --use_cuda
            OrtCUDAProviderOptions cuda{};
            cuda.device_id = 0;
            try {
                opts.AppendExecutionProvider_CUDA(cuda);
                gpu_provider_ = GpuProvider::CUDA;
            } catch (const Ort::Exception&) {}

            // Second, if CUDA is not available, we try ROCm. If it also fails, we simply continue with CPU execution.
            // Needs onnxruntime built with --use_rocm
            if (gpu_provider_ == GpuProvider::None) {
                OrtROCMProviderOptions rocm{};
                try {
                    opts.AppendExecutionProvider_ROCM(rocm);
                    gpu_provider_ = GpuProvider::ROCm;
                } catch (const Ort::Exception&) { /* ROCm absent → CPU */ }
            }

            // If no GPU provider was successfully appended, gpu_provider_ remains GpuProvider::None, and the session will run on CPU by default.
        }

        session_ = std::make_unique<Ort::Session>(ort_env_, model_path.c_str(), opts);
    }

    [[nodiscard]] GpuProvider gpuProvider()  const noexcept { return gpu_provider_; }
    [[nodiscard]] bool gpuEnabled() const noexcept { return gpu_provider_ != GpuProvider::None; }

    [[nodiscard]] std::string_view providerName() const noexcept
    {
        switch (gpu_provider_) {
            case GpuProvider::CUDA: return "CUDA (Nvidia)";
            case GpuProvider::ROCm: return "ROCm (AMD)";
            default:                return "CPU";
        }
    }

    /**
     * Run inference on the given BGR frame and return a vector of detections.
     * The input frame is preprocessed (resized with letterbox, color converted, normalized)
     * before being fed to the model, and the raw output is post-processed to extract bounding boxes,
     * confidence scores, and class IDs.
     *
     * @param bgr_frame Input image in BGR format (as typically used by OpenCV).
     * @return A vector of Detection objects representing the detected objects in the original image space.
     */
    [[nodiscard]] std::vector<Detection> detect(const cv::Mat& bgr_frame)
    {
        LetterboxParams lb;
        auto input_data = preprocess(bgr_frame, lb);
        auto raw_output = runInference(input_data);
        return postprocess(raw_output, lb);
    }

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
    [[nodiscard]] std::vector<float> preprocess(const cv::Mat& src, LetterboxParams& lb) const
    {
        lb.scale = std::min(
            YOLO_INPUT_SIZE / static_cast<float>(src.cols),
            YOLO_INPUT_SIZE / static_cast<float>(src.rows)
        );

        const int new_w = static_cast<int>(src.cols * lb.scale);
        const int new_h = static_cast<int>(src.rows * lb.scale);
        lb.pad_x = (YOLO_INPUT_SIZE - new_w) / 2.0f;
        lb.pad_y = (YOLO_INPUT_SIZE - new_h) / 2.0f;

        cv::Mat resized;
        cv::resize(src, resized, {new_w, new_h}, 0, 0, cv::INTER_LINEAR);

        cv::Mat padded(YOLO_INPUT_SIZE, YOLO_INPUT_SIZE, CV_8UC3, cv::Scalar(LETTERBOX_FILL, LETTERBOX_FILL, LETTERBOX_FILL));
        resized.copyTo(padded(cv::Rect(
            static_cast<int>(lb.pad_x), static_cast<int>(lb.pad_y),
            new_w, new_h
        )));

        cv::Mat rgb;
        cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);

        static constexpr int CH_SIZE = YOLO_INPUT_SIZE * YOLO_INPUT_SIZE;
        std::vector<float> tensor(3 * CH_SIZE);

        for (int y = 0; y < YOLO_INPUT_SIZE; ++y) {
            const auto* row = rgb.ptr<uint8_t>(y);
            for (int x = 0; x < YOLO_INPUT_SIZE; ++x) {
                const int idx = y * YOLO_INPUT_SIZE + x;
                tensor[0 * CH_SIZE + idx] = row[x * 3 + 0] / 255.0f;  // R
                tensor[1 * CH_SIZE + idx] = row[x * 3 + 1] / 255.0f;  // G
                tensor[2 * CH_SIZE + idx] = row[x * 3 + 2] / 255.0f;  // B
            }
        }

        return tensor;
    }

    /**
     * Run inference on the preprocessed input tensor using the ONNX Runtime session.
     * The function prepares the input tensor in the format expected by the model, executes the session, and retrieves the raw output tensor as a vector of floats.
     * The output is expected to be in the shape [1, 84, 8400], where 84 corresponds to the 4 bounding box parameters (x_center, y_center, width, height)
     * plus 80 class confidence scores, and 8400 corresponds to the number of anchor points for a 640x640 input.
     *
     * @param input A vector of floats representing the preprocessed image tensor in CHW format.
     * @return A vector of floats containing the raw output from the model, which will be post-processed to extract detections.
     */
    [[nodiscard]] std::vector<float> runInference(const std::vector<float>& input) const
    {
        constexpr std::array<int64_t, 4> shape{1, 3, YOLO_INPUT_SIZE, YOLO_INPUT_SIZE};

        auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        auto in_tensor = Ort::Value::CreateTensor<float>(
            mem_info,
            const_cast<float*>(input.data()), input.size(),
            shape.data(), shape.size()
        );

        const char* in_names[]  = {"images"};
        const char* out_names[] = {"output0"};

        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            in_names, &in_tensor, 1,
            out_names, 1
        );

        const auto* ptr  = outputs[0].GetTensorData<float>();
        const auto  info = outputs[0].GetTensorTypeAndShapeInfo();
        return std::vector<float>(ptr, ptr + info.GetElementCount());
    }

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
    [[nodiscard]] std::vector<Detection> postprocess(const std::vector<float>& output, const LetterboxParams&   lb) const
    {
        std::vector<Detection> candidates;

        for (int i = 0; i < YOLO_NUM_ANCHORS; ++i) {
            const float conf = output[(4 + YOLO_CLASS_PERSON) * YOLO_NUM_ANCHORS + i];
            if (conf < conf_thresh_) continue;

            const float cx_lb = output[0 * YOLO_NUM_ANCHORS + i];
            const float cy_lb = output[1 * YOLO_NUM_ANCHORS + i];
            const float w_lb  = output[2 * YOLO_NUM_ANCHORS + i];
            const float h_lb  = output[3 * YOLO_NUM_ANCHORS + i];

            candidates.push_back({
                .cx         = (cx_lb - lb.pad_x) / lb.scale,
                .cy         = (cy_lb - lb.pad_y) / lb.scale,
                .w          = w_lb / lb.scale,
                .h          = h_lb / lb.scale,
                .confidence = conf,
                .class_id   = YOLO_CLASS_PERSON
            });
        }

        return applyNMS(candidates);
    }

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
    [[nodiscard]] static float iou(const Detection& a, const Detection& b) noexcept
    {
        const float ax1 = a.cx - a.w / 2.0f,  ay1 = a.cy - a.h / 2.0f;
        const float ax2 = a.cx + a.w / 2.0f,  ay2 = a.cy + a.h / 2.0f;
        const float bx1 = b.cx - b.w / 2.0f,  by1 = b.cy - b.h / 2.0f;
        const float bx2 = b.cx + b.w / 2.0f,  by2 = b.cy + b.h / 2.0f;

        const float ix1 = std::max(ax1, bx1),  iy1 = std::max(ay1, by1);
        const float ix2 = std::min(ax2, bx2),  iy2 = std::min(ay2, by2);
        const float inter = std::max(0.0f, ix2 - ix1) * std::max(0.0f, iy2 - iy1);

        if (inter == 0.0f) return 0.0f;

        const float area_a = a.w * a.h;
        const float area_b = b.w * b.h;
        return inter / (area_a + area_b - inter);
    }

    /**
     * Apply Non-Maximum Suppression (NMS) to a list of candidate detections to filter out duplicates.
     * The function first sorts the detections by confidence score in descending order,
     * then iteratively selects the highest-confidence detection and suppresses any subsequent detections that have an IoU greater than the specified threshold with the selected detection.
     * The result is a vector of Detection objects that are considered valid and non-overlapping according to the NMS criteria.
     *
     * @param dets A vector of Detection objects representing candidate detections before NMS.
     * @return A vector of Detection objects that have been filtered by NMS, containing only the most confident detections without significant overlap.
     */
    [[nodiscard]] std::vector<Detection> applyNMS(std::vector<Detection> dets) const
    {
        std::sort(dets.begin(), dets.end(), [](const Detection& a, const Detection& b) { return a.confidence > b.confidence; });

        std::vector<bool> suppressed(dets.size(), false);
        std::vector<Detection> result;

        for (std::size_t i = 0; i < dets.size(); ++i) {
            if (suppressed[i]) continue;

            result.push_back(dets[i]);

            for (std::size_t j = i + 1; j < dets.size(); ++j) {
                if (!suppressed[j] && iou(dets[i], dets[j]) > nms_iou_) {
                    suppressed[j] = true;
                }
            }
        }

        return result;
    }

    float conf_thresh_{YOLO_DEFAULT_CONF};
    float nms_iou_{YOLO_DEFAULT_NMS_IOU};
    GpuProvider gpu_provider_{GpuProvider::None};
    Ort::Env ort_env_;
    std::unique_ptr<Ort::Session> session_;
};

}