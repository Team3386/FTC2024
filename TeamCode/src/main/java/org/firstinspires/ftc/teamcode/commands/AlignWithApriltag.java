package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AlignWithApriltag extends CommandBase {
    private final DriveSubsystem robotDrive = DriveSubsystem.getInstance();
    private final VisionSubsystem robotVision = VisionSubsystem.getInstance();
    private final PIDFController controller;
    int withoutDetection = 0;
    int onPoint = 0;
    int targetID;

    public AlignWithApriltag(int target) {
        this(
                target,
                new PIDFController(
                        Constants.AutonomousConstants.APRILTAG_PID[0],
                        Constants.AutonomousConstants.APRILTAG_PID[1],
                        Constants.AutonomousConstants.APRILTAG_PID[2],
                        0
                )
        );
    }

    public AlignWithApriltag(int target, PIDFController pid) {
        Log.i("AlignWithAprilTag", String.format("Initializing with target %d", target));
        controller = pid;
        controller.setTolerance(0.6);
        controller.setSetPoint(0);
        controller.calculate(99999999);
        targetID = target;
    }

    @Override
    public void initialize() {
        robotDrive.autoOverride = true;
    }

    @Override
    public void execute() {
        if (!robotVision.gotNewAprilTag()) {
            withoutDetection++;
            return;
        }
        withoutDetection = 0;

        Log.i("AlignWithAprilTag", String.format("Current AprilTag offset %f", robotVision.aprilTagPose().x));

        double xSpeed = -controller.calculate(robotVision.aprilTagPose().x, 0);
        xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), Constants.AutonomousConstants.APRILTAG_SPEED_LIMIT), xSpeed);
        robotDrive.drive(new ChassisSpeeds(xSpeed, 0, 0), false, new Rotation2d());
    }

    @Override
    public boolean isFinished() {
        if (controller.atSetPoint()) {
            onPoint++;
        } else {
            onPoint = 0;
        }

        if (onPoint > 10) {
            Log.i("AlignWithAprilTag", "Exiting as controller arrived at setpoint");
            Pose2d currentPose = OdometrySubsystem.getInstance().getPose();
            double aprilTagX = Constants.VisionConstants.APRILTAG_POSITIONS.get(targetID).getX();
            OdometrySubsystem.getInstance().resetPose(new Pose2d(aprilTagX, currentPose.getY(), currentPose.getRotation()));
        }
        if (withoutDetection > 100) {
            Log.i("AlignWithAprilTag", "Exiting as no AprilTag is detected");
        }
        return withoutDetection > 100 || onPoint > 10;
    }

    @Override
    public void end(boolean interrupted) {
        robotDrive.autoOverride = false;
    }
}
