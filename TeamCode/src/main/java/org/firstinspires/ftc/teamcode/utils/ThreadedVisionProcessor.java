package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Canvas;

import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.utils.ncnnprocessor.NcnnProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

public class ThreadedVisionProcessor implements VisionProcessor {
    public final AprilTagProcessor aprilTagProcessor;
    public final NcnnProcessor ncnnProcessor;
    private boolean aprilTagLockedForAccess;
    private boolean ncnnLockedForAccess;
    private Future<Object> aprilTagFuture;
    private Future<Object> ncnnFuture;

    public ThreadedVisionProcessor(AprilTagProcessor aprilTag, NcnnProcessor ncnn) {
        aprilTagProcessor = aprilTag;
        ncnnProcessor = ncnn;
        aprilTagFuture = CompletableFuture.completedFuture(null);
        ncnnFuture = CompletableFuture.completedFuture(null);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        aprilTagProcessor.init(width, height, calibration);
        ncnnProcessor.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        ExecutorService threadPool = ThreadPool.getDefault();

        if (!aprilTagLockedForAccess && aprilTagFuture.isDone()) {
            aprilTagLockedForAccess = true;
            aprilTagFuture = threadPool.submit(() -> aprilTagProcessor.processFrame(frame, captureTimeNanos));
        }

        if (!ncnnLockedForAccess && ncnnFuture.isDone()) {
            ncnnLockedForAccess = true;
            ncnnFuture = threadPool.submit(() -> ncnnProcessor.processFrame(frame, captureTimeNanos));
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (aprilTagFuture.isDone()) {
            aprilTagProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
        }

        if (ncnnFuture.isDone()) {
            ncnnProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
        }
    }

    public boolean canAccessAprilTag() {
        if (aprilTagLockedForAccess && aprilTagFuture.isDone()) {
            aprilTagLockedForAccess = false;
            return true;
        }
        return false;
    }

    public boolean canAccessNcnn() {
        if (ncnnLockedForAccess && ncnnFuture.isDone()) {
            ncnnLockedForAccess = false;
            return true;
        }
        return false;
    }
}
