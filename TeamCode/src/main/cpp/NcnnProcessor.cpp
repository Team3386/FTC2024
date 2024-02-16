#include <android/log.h>
#include <cpu.h>
#include "NcnnProcessor.h"

NcnnProcessor::NcnnProcessor(AAssetManager *mgr,
                             const char *paramAssetPath,
                             const char *modelAssetPath,
                             std::array<int, 2> &inputSize,
                             std::array<float, 3> &meanValues,
                             std::array<float, 3> &normValues,
                             float confidenceThreshold, int threadNum) :
        inputSize(inputSize), meanValues(meanValues), normValues(normValues),
        confidenceThreshold(confidenceThreshold) {
    this->net.clear();
    this->blobPoolAllocator.clear();
    this->workspacePoolAllocator.clear();

//    this->blobPoolAllocator.set_size_compare_ratio(0.f);
//    this->workspacePoolAllocator.set_size_compare_ratio(0.f);

    ncnn::set_omp_num_threads(threadNum);

    this->net.opt = ncnn::Option();
    this->net.opt.num_threads = threadNum;
    this->net.opt.blob_allocator = &this->blobPoolAllocator;
    this->net.opt.workspace_allocator = &this->workspacePoolAllocator;
    this->net.opt.use_bf16_storage = true;
    this->net.opt.use_packing_layout = true;

    this->net.load_param(mgr, paramAssetPath);
    this->net.load_model(mgr, modelAssetPath);

    this->softmax = ncnn::create_layer("Softmax");

    ncnn::ParamDict pd;
    pd.set(0, 1); // axis
    pd.set(1, 1);
    this->softmax->load_param(pd);

    this->softmax_opt = ncnn::Option();
    this->softmax_opt.num_threads = 1;
    this->softmax_opt.use_packing_layout = false;

    softmax->create_pipeline(this->softmax_opt);
}

NcnnProcessor::~NcnnProcessor() {
    delete this->softmax;
}

void NcnnProcessor::init(int frameWidth, int frameHeight) {
    this->scaledSize = this->inputSize;
    if (frameWidth > frameHeight) {
        const float scale = (float) this->inputSize[0] / frameWidth;
        scaledSize[1] = frameHeight * scale;
    } else {
        const float scale = (float) this->inputSize[1] / frameHeight;
        scaledSize[0] = frameWidth * scale;
    }

    int paddingWidth = (this->scaledSize[0] + 31) / 32 * 32 - this->scaledSize[0];
    int paddingHeight = (this->scaledSize[1] + 31) / 32 * 32 - this->scaledSize[1];

    this->paddingSize = {paddingHeight / 2, paddingHeight - paddingHeight / 2, paddingWidth / 2,
                         paddingWidth - paddingWidth / 2};
}

void NcnnProcessor::processFrame(cv::Mat frame) {
    double t = now_ms();

    ncnn::Mat resizedFrame = ncnn::Mat::from_pixels_resize(
            frame.data, ncnn::Mat::PIXEL_RGBA2BGR,
            frame.cols, frame.rows,
            scaledSize[0], scaledSize[1]
    );

    ncnn::Mat paddedFrame;
    ncnn::copy_make_border(resizedFrame, paddedFrame, this->paddingSize[0], this->paddingSize[1],
                           this->paddingSize[2], this->paddingSize[3], ncnn::BORDER_CONSTANT, 0.f);

    paddedFrame.substract_mean_normalize(this->meanValues.data(), this->normValues.data());

    ncnn::Extractor extractor = this->net.create_extractor();

    extractor.input("input.1", paddedFrame);

    __android_log_print(ANDROID_LOG_INFO, "NativeNcnnProcessor",
                        "Frame preprocessing took %f ms",
                        now_ms() - t
    );

    t = now_ms();

    this->detections.clear();

    // stride 8
    {
        ncnn::Mat clsPred;
        ncnn::Mat disPred;
        extractor.extract("cls_pred_stride_8", clsPred);
        extractor.extract("dis_pred_stride_8", disPred);

        this->processResults(clsPred, disPred, 8, paddedFrame);
    }

    // stride 16
    {
        ncnn::Mat clsPred;
        ncnn::Mat disPred;
        extractor.extract("cls_pred_stride_16", clsPred);
        extractor.extract("dis_pred_stride_16", disPred);

        this->processResults(clsPred, disPred, 16, paddedFrame);
    }

    // stride 32
    {
        ncnn::Mat clsPred;
        ncnn::Mat disPred;
        extractor.extract("cls_pred_stride_32", clsPred);
        extractor.extract("dis_pred_stride_32", disPred);

        this->processResults(clsPred, disPred, 32, paddedFrame);
    }

    __android_log_print(ANDROID_LOG_INFO, "NativeNcnnProcessor",
                        "Inference took %f ms",
                        now_ms() - t
    );

    if (!this->detections.empty()) {
        __android_log_print(ANDROID_LOG_INFO, "NativeNcnnProcessor",
                            "Got detections"
        );
        for (DetectedObject object: this->detections)
            __android_log_print(ANDROID_LOG_INFO, "NativeNcnnProcessor",
                                "DetectedObject: Label %s(%i), Confidence %f",
                                this->CLASS_NAMES[object.label], object.label,
                                object.confidence
            );
    }
}

void NcnnProcessor::processResults(const ncnn::Mat &clsPred, const ncnn::Mat &disPred, int stride,
                                   const ncnn::Mat &input) {
    const int num_grid = clsPred.h;

    int num_grid_x;
    int num_grid_y;
    if (input.w > input.h) {
        num_grid_x = input.w / stride;
        num_grid_y = num_grid / num_grid_x;
    } else {
        num_grid_y = input.h / stride;
        num_grid_x = num_grid / num_grid_y;
    }

    const int numClass = clsPred.w;
    const int reg_max_1 = disPred.w / 4;

    for (int i = 0; i < num_grid_y; i++) {
        for (int j = 0; j < num_grid_x; j++) {
            const int idx = i * num_grid_x + j;

            const float *scores = clsPred.row(idx);

            // find label with max score
            int label = -1;
            float score = -FLT_MAX;
            for (int k = 0; k < numClass; k++) {
                if (scores[k] > score) {
                    label = k;
                    score = scores[k];
                }
            }

            if (score >= this->confidenceThreshold) {
                ncnn::Mat bbox_pred(reg_max_1, 4, (void *) disPred.row(idx));
                this->softmax->forward_inplace(bbox_pred, this->softmax_opt);

                float pred_ltrb[4];
                for (int k = 0; k < 4; k++) {
                    float dis = 0.f;
                    const float *dis_after_sm = bbox_pred.row(k);
                    for (int l = 0; l < reg_max_1; l++) {
                        dis += l * dis_after_sm[l];
                    }

                    pred_ltrb[k] = dis * stride;
                }

                float pb_cx = (j + 0.5f) * stride;
                float pb_cy = (i + 0.5f) * stride;

                float x0 = pb_cx - pred_ltrb[0];
                float y0 = pb_cy - pred_ltrb[1];
                float x1 = pb_cx + pred_ltrb[2];
                float y1 = pb_cy + pred_ltrb[3];

                this->detections.push_back(
                        DetectedObject(cv::Rect2f(x0, y0, x1 - x0, y1 - y0), label, score));
            }
        }
    }
}