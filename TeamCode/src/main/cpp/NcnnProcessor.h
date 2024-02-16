#ifndef NCNNPROCESSOR_H
#define NCNNPROCESSOR_H

#include "opencv2/opencv.hpp"
#include <net.h>

#include "DetectedObject.h"

class NcnnProcessor {
public:
    NcnnProcessor(AAssetManager *mgr,
                  const char *paramAssetPath,
                  const char *modelAssetPath,
                  std::array<int, 2> &inputSize,
                  std::array<float, 3> &meanValues,
                  std::array<float, 3> &normValues,
                  float confidenceThreshold, int threadNum);

    ~NcnnProcessor();

    void init(int frameWidth, int frameHeight);

    void processFrame(cv::Mat frame);

    void processResults(const ncnn::Mat &clsPred, const ncnn::Mat &disPred, int stride,
                        const ncnn::Mat &input);

    ncnn::Net net;
    ncnn::UnlockedPoolAllocator blobPoolAllocator;
    ncnn::PoolAllocator workspacePoolAllocator;

    ncnn::Layer *softmax;
    ncnn::Option softmax_opt;

    const std::array<int, 2> inputSize; // input size
    std::array<int, 2> scaledSize; // scaled height and width
    std::array<int, 4> paddingSize; // Top bottom left right of image padding
    const std::array<float, 3> meanValues;
    const std::array<float, 3> normValues;
    const float confidenceThreshold;

    int num_class = 80; // number of classes. 80 for COCO
    int reg_max = 7; // `reg_max` set in the training config. Default: 7.
    std::vector<int> strides = {8, 16, 32, 64}; // strides of the multi-level feature.

    std::vector<DetectedObject> detections;

    const char *CLASS_NAMES[80] = {
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light",
            "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
            "sheep", "cow",
            "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie",
            "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
            "skateboard", "surfboard",
            "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
            "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
            "chair", "couch",
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
            "keyboard", "cell phone",
            "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
            "scissors", "teddy bear",
            "hair drier", "toothbrush"
    };
};

#include <time.h>

// from android samples
/* return current time in milliseconds */
static double now_ms(void) {

    struct timespec res;
    clock_gettime(CLOCK_REALTIME, &res);
    return 1000.0 * res.tv_sec + (double) res.tv_nsec / 1e6;

}

#endif // NCNNPROCESSOR_H
