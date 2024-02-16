package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;

public class ObjectProcessor implements VisionProcessor {
    private static final Mat GREY = new Mat();
    private final CascadeClassifier cascadeClassifier = new CascadeClassifier();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        cascadeClassifier.load("cascade.xml");
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, GREY, Imgproc.COLOR_RGBA2GRAY);
//        Imgproc.equalizeHist(frame, frame);
        MatOfRect objects = new MatOfRect();
        cascadeClassifier.detectMultiScale(frame, objects);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
