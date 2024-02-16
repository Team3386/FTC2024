package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.ObjectProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    private static final VisionSubsystem INSTANCE = new VisionSubsystem();

    private VisionPortal frontWebcam;
    private VisionPortal rearWebcam;
    private AprilTagProcessor frontAprilTag;
    private ObjectProcessor frontObject;
    private AprilTagProcessor rearAprilTag;

    private VisionSubsystem() {
    }

    public static VisionSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;

        frontAprilTag = new AprilTagProcessor.Builder().build();
        rearAprilTag = new AprilTagProcessor.Builder().build();
        frontObject = new ObjectProcessor();

        VisionPortal.Builder frontBuilder = new VisionPortal.Builder();
        frontBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        frontBuilder.addProcessor(frontAprilTag);
        frontBuilder.addProcessor(frontObject);
        frontWebcam = frontBuilder.build();

//        VisionPortal.Builder rearBuilder = new VisionPortal.Builder();
//        rearBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
//        rearBuilder.addProcessor(rearAprilTag);
//        rearWebcam = rearBuilder.build();
    }

    public void periodic() {
        Telemetry telemetry = GlobalSubsystem.getInstance().telemetry;

        telemetry.addLine("Front AprilTags");
        logAprilTagDetections(frontAprilTag.getDetections());

        telemetry.addLine("Rear AprilTags");
        logAprilTagDetections(rearAprilTag.getDetections());
    }

    public void logAprilTagDetections(List<AprilTagDetection> detections) {
        Telemetry telemetry = GlobalSubsystem.getInstance().telemetry;

        telemetry.addData("# AprilTags Detected", detections.size());

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
}
