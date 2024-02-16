package org.firstinspires.ftc.teamcode.utils.ncnnprocessor;

import org.opencv.core.Rect2d;

public class DetectedObject {
    Rect2d rect;
    int label;
    float confidence;

    DetectedObject(Rect2d rect, int label, float confidence) {
        this.rect = rect;
        this.label = label;
        this.confidence = confidence;
    }
}
