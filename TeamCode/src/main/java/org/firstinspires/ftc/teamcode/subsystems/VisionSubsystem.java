package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.teamcode.utils.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.utils.TempProcessor;
import org.firstinspires.ftc.teamcode.utils.ncnnprocessor.DetectedObject;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private static final VisionSubsystem INSTANCE = new VisionSubsystem();
    private final List<DetectedObject> ncnnResults = new ArrayList<>();
    public int propPos = 0;
    private VisionPortal frontWebcam;
    private TempProcessor processors;
    private CameraStreamProcessor stream;
    private List<AprilTagDetection> aprilTagResults = new ArrayList<>();
    private List<Recognition> tfResults = new ArrayList<>();
    private Translation2d lastEstimatedPosition = new Translation2d();
    private AprilTagPoseFtc lastAprilTagPose;
    private boolean gotNewDetection = false;
    private boolean fixProp;

    private VisionSubsystem() {
    }

    public static VisionSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;

        AprilTagProcessor frontAprilTag = new AprilTagProcessor.Builder().setNumThreads(1).build();
        frontAprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SQPNP);

//        NcnnProcessor ncnnProcessor = new NcnnProcessor(
//                "models/nanodet_m-int8.param", "models/nanodet_m-int8.bin",
//                new int[]{320, 320}, new float[]{103.53f, 116.28f, 123.675f},
//                new float[]{1.f / 57.375f, 1.f / 57.12f, 1.f / 58.395f},
//                0.5f,
//                1
//        );

        TfodProcessor tf = new TfodProcessor.Builder()
                .setModelFileName("Prop.tflite")
                .setModelLabels(new String[]{"Prop"}).build();

        processors = new TempProcessor(frontAprilTag, tf);
        stream = new CameraStreamProcessor();

        VisionPortal.Builder frontBuilder = new VisionPortal.Builder();
        frontBuilder
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processors)
                .addProcessor(stream);
        frontWebcam = frontBuilder.build();

        FtcDashboard.getInstance().startCameraStream(stream, 0);

//        VisionPortal.Builder rearBuilder = new VisionPortal.Builder();
//        rearBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
//        rearBuilder.addProcessor(rearAprilTag);
//        rearWebcam = rearBuilder.build();
        frontWebcam.resumeStreaming();
        fixProp = false;
    }

    @Override
    public void periodic() {
        if (processors.canAccessAprilTag()) {
            aprilTagResults = new ArrayList<>(processors.aprilTagProcessor.getDetections());
            OdometrySubsystem robotOdometry = OdometrySubsystem.getInstance();
            if (!aprilTagResults.isEmpty()) {
                Optional<AprilTagDetection> detected = aprilTagResults.stream()
                        .filter(det -> VisionConstants.APRILTAG_POSITIONS.containsKey(det.id)).findFirst();
                if (detected.isPresent()) {
                    gotNewDetection = true;
                    lastAprilTagPose = detected.get().ftcPose;
                    Translation2d computedPosition = computePosition(detected.get());
                    if (computedPosition != null) {
                        lastEstimatedPosition = computedPosition;
//                        robotOdometry.visionResetPosition(computedPosition);
                    }
                } else {
                    gotNewDetection = false;
                }
            }
            robotOdometry.resetVisionStore();
        }

        TelemetryPacket fieldPacket = GlobalSubsystem.getInstance().fieldPacket;

        fieldPacket.fieldOverlay().setTranslation(lastEstimatedPosition.getY() * Constants.CENTIMETER_PER_INCH_INVERSE, -lastEstimatedPosition.getX() * Constants.CENTIMETER_PER_INCH_INVERSE);

        fieldPacket.fieldOverlay()
                .setStroke("green")
                .setRotation(0)
                .strokeRect(-Constants.DriveConstants.TRACK_WIDTH / 2, -Constants.DriveConstants.WHEEL_BASE / 2, Constants.DriveConstants.TRACK_WIDTH, Constants.DriveConstants.WHEEL_BASE);

        fieldPacket.fieldOverlay().setTranslation(0, 0);
        for (Translation2d position : VisionConstants.APRILTAG_POSITIONS.values()) {
            fieldPacket.fieldOverlay().strokeCircle(position.getY(), -position.getX(), 2);
        }

        if (processors.canAccessTf()) {
            tfResults = new ArrayList<>(processors.tfProcessor.getRecognitions());
        }

        for (Recognition rec : tfResults) {
            GlobalSubsystem.getInstance().telemetry.addData(String.format("Vision: Detected object", rec.getWidth()), rec);
        }

        // NOTE: The Y coordinates of the results are flipped
        Optional<Recognition> firstRec = tfResults.stream()
                .max(Comparator.comparingDouble(rec -> rec.getBottom() - rec.getHeight() / 2));

        if (firstRec.isPresent() && !fixProp) {
            Recognition recognition = firstRec.get();
            final float x = (recognition.getLeft() + recognition.getWidth() / 2);
            GlobalSubsystem.getInstance().telemetry.addData("Vision: Detected prop center", x);
            float third = 640f / 3f;
            propPos = (int) (x / third);
        }

        GlobalSubsystem.getInstance().telemetry.addData("Vision: Prop position", propPos);

        logAprilTagDetections(aprilTagResults);
    }

    private Translation2d computePosition(AprilTagDetection detection) {
        Translation2d aprilTagPosition = VisionConstants.APRILTAG_POSITIONS.get(detection.id);
        if (aprilTagPosition == null) {
            return null;
        }

        Pose2d robotPose = OdometrySubsystem.getInstance().getPose();

        TelemetryPacket packet = GlobalSubsystem.getInstance().fieldPacket;

        Translation2d tagRelativeToCamera = new Translation2d(detection.ftcPose.x, detection.ftcPose.y * Constants.CENTIMETER_PER_INCH);
        Translation2d tagRelativeToRobot = tagRelativeToCamera.minus(VisionConstants.CAMERA_POSITION);
        Translation2d tagRelativeToRobotGlobal = tagRelativeToRobot.rotateBy(robotPose.getRotation().unaryMinus());

        packet.fieldOverlay().setTranslation(robotPose.getY(), -robotPose.getX()).setRotation(0);
        packet.fieldOverlay().strokeLine(0, 0, tagRelativeToRobotGlobal.getY(), -tagRelativeToRobotGlobal.getX());

        Translation2d robotRelativeToTag = tagRelativeToRobotGlobal.unaryMinus();

        packet.fieldOverlay().setTranslation(aprilTagPosition.getY(), -aprilTagPosition.getX()).setRotation(0);
        packet.fieldOverlay().strokeLine(0, 0, robotRelativeToTag.getY(), -robotRelativeToTag.getX());

        if (Math.abs(robotRelativeToTag.getNorm()) > 100 || Math.abs(robotRelativeToTag.getNorm()) < 60) {
            return null;
        }

        return aprilTagPosition.plus(robotRelativeToTag);
    }

    public void logAprilTagDetections(List<AprilTagDetection> detections) {
        Telemetry telemetry = GlobalSubsystem.getInstance().telemetry;

        telemetry.addData("Vision: # AprilTags Detected", detections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }

    public void lockProp() {
        fixProp = true;
        processors.tfEnabled = false;
    }

    public AprilTagPoseFtc aprilTagPose() {
        return lastAprilTagPose;
    }

    public boolean gotNewAprilTag() {
        return gotNewDetection;
    }
}
