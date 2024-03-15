package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Canvas;

import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

public class TempProcessor implements VisionProcessor {
    public final AprilTagProcessor aprilTagProcessor;
    public final TfodProcessor tfProcessor;
    private final Future<Object> aprilTagFuture;
    private final Future<Object> tfFuture;
    private boolean aprilTagLockedForAccess;
    private boolean tfLockedForAccess;

    public TempProcessor(AprilTagProcessor aprilTag, TfodProcessor tf) {
        aprilTagProcessor = aprilTag;
        tfProcessor = tf;
        aprilTagFuture = CompletableFuture.completedFuture(null);
        tfFuture = CompletableFuture.completedFuture(null);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        aprilTagProcessor.init(width, height, calibration);
        tfProcessor.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        ExecutorService threadPool = ThreadPool.getDefault();

//        if (!aprilTagLockedForAccess && aprilTagFuture.isDone()) {
//            aprilTagLockedForAccess = true;
//            aprilTagFuture = threadPool.submit(() -> aprilTagProcessor.processFrame(frame, captureTimeNanos));
//        }
//
//        if (!tfLockedForAccess && tfFuture.isDone()) {
//            tfLockedForAccess = true;
//            tfFuture = threadPool.submit(() -> tfProcessor.processFrame(frame, captureTimeNanos));
//        }

        aprilTagLockedForAccess = true;
        tfLockedForAccess = true;
        tfProcessor.processFrame(frame, captureTimeNanos);
        aprilTagProcessor.processFrame(frame, captureTimeNanos);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        if (aprilTagFuture.isDone()) {
//            aprilTagProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
//        }
        aprilTagProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
        tfProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
//        if (tfFuture.isDone()) {
//            tfProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
//        }
    }

    public boolean canAccessAprilTag() {
        if (aprilTagLockedForAccess && aprilTagFuture.isDone()) {
            aprilTagLockedForAccess = false;
            return true;
        }
        return false;
    }

    public boolean canAccessTf() {
        if (tfLockedForAccess && tfFuture.isDone()) {
            tfLockedForAccess = false;
            return true;
        }
        return false;
    }
}
