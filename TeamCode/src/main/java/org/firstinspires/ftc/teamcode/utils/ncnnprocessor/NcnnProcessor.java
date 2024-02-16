package org.firstinspires.ftc.teamcode.utils.ncnnprocessor;

import android.content.res.AssetManager;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class NcnnProcessor implements VisionProcessor {
    static {
        System.loadLibrary("ncnn_processor");
    }

    public NcnnProcessor(String paramPath, String modelPath, int[] inputSize, float[] meanValues, float[] normValues, float confidenceThreshold, int threadNum) {
        CreateRawInstance(AppUtil.getDefContext().getAssets(), paramPath, modelPath, inputSize, meanValues, normValues, confidenceThreshold, threadNum);
    }

    private native void CreateRawInstance(AssetManager assetManager, String paramPath, String modelPath, int[] inputSize, float[] meanValues, float[] normValues, float confidenceThreshold, int threadNum);

    private native void RawInit(int frameWidth, int frameHeight);

    private native void RawProcessFrame(long frameAddress);

    private native DetectedObject[] RawGetDetections();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        RawInit(width, height);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        RawProcessFrame(frame.getNativeObjAddr());
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public DetectedObject[] getDetections() {
        return RawGetDetections();
    }
}
