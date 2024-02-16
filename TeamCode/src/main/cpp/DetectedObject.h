#ifndef DETECTEDOBJECT_H
#define DETECTEDOBJECT_H

struct DetectedObject {
    cv::Rect2f rect;
    int label;
    float confidence;

    DetectedObject(cv::Rect2f rect, int label, float confidence) : rect(rect), label(label),
                                                                   confidence(confidence) {};
};

#endif // DETECTEDOBJECT_H
