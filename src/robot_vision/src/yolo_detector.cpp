#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>

#include "robot_vision/yolo_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>

namespace robot_vision
{

YoloDetector::YoloDetector(const std::filesystem::path& model_path, bool use_gpu, float conf_thresh, float nms_iou)
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

[[nodiscard]] GpuProvider YoloDetector::gpuProvider()  const noexcept { return gpu_provider_; }
[[nodiscard]] bool YoloDetector::gpuEnabled() const noexcept { return gpu_provider_ != GpuProvider::None; }

[[nodiscard]] std::string_view YoloDetector::providerName() const noexcept
{
    switch (gpu_provider_) {
        case GpuProvider::CUDA: return "CUDA (Nvidia)";
        case GpuProvider::ROCm: return "ROCm (AMD)";
        default:                return "CPU";
    }
}

[[nodiscard]] std::vector<Detection> YoloDetector::detect(const cv::Mat& bgr_frame)
{
    LetterboxParams lb;
    auto input_data = preprocess(bgr_frame, lb);
    auto raw_output = runInference(input_data);
    return postprocess(raw_output, lb);
}

[[nodiscard]] std::vector<float> YoloDetector::preprocess(const cv::Mat& src, LetterboxParams& lb) const
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

[[nodiscard]] std::vector<float> YoloDetector::runInference(const std::vector<float>& input) const
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

[[nodiscard]] std::vector<Detection> YoloDetector::postprocess(const std::vector<float>& output, const LetterboxParams& lb) const
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

[[nodiscard]] float YoloDetector::iou(const Detection& a, const Detection& b) noexcept
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

[[nodiscard]] std::vector<Detection> YoloDetector::applyNMS(std::vector<Detection> dets) const
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

} // namespace robot_vision
