package org.firstinspires.ftc.teamcode.commands.purepursuit;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotMath;
import org.firstinspires.ftc.teamcode.utils.RotationPIDFController;

import java.util.List;

// References:
// https://www.roboticsisez.com/projects/pure-pursuit/
// https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
// https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
// https://github.com/Dawgma-1712/new-FRC-2018
// https://github.com/Team488/SeriouslyCommonLib/blob/b3cb/src/main/java/xbot/common/subsystems/drive/PurePursuitCommand.java
// https://github.com/Team254/FRC-2019-Public/blob/576b/src/main/java/com/team254/lib/control/AdaptivePurePursuitController.java

public class PurePursuitCommand extends CommandBase {
    private static final DriveSubsystem robotDrive = DriveSubsystem.getInstance();
    private static final OdometrySubsystem robotOdometry = OdometrySubsystem.getInstance();

    private final PIDFController xController;
    private final PIDFController yController;
    private final RotationPIDFController rotationController;

    // How close should the robot be to the current point to select the next one
    private final double advancePointThreshold = 10;
    // How close should the robot be to start rotating towards the target angle
    private final double lookAtTargetThreshold = 30;

    private final List<Point> pointsToVisit;
    private final boolean disableFastDrive;
    private int pointIndex = 0;

    public PurePursuitCommand(List<Point> pointsToVisit) {
        this(
                new PIDFController(
                        Constants.AutonomousConstants.POSITION_PID[0],
                        Constants.AutonomousConstants.POSITION_PID[1],
                        Constants.AutonomousConstants.POSITION_PID[2],
                        0
                ),
                new PIDFController(
                        Constants.AutonomousConstants.POSITION_PID[0],
                        Constants.AutonomousConstants.POSITION_PID[1],
                        Constants.AutonomousConstants.POSITION_PID[2],
                        0
                ),
                new RotationPIDFController(
                        Constants.AutonomousConstants.ROTATION_PID[0],
                        Constants.AutonomousConstants.ROTATION_PID[1],
                        Constants.AutonomousConstants.ROTATION_PID[2],
                        0
                ),
                pointsToVisit
        );
    }

    public PurePursuitCommand(PIDFController xController, PIDFController yController, RotationPIDFController rotationController,
                              List<Point> pointsToVisit) {
        this(xController, yController, rotationController, pointsToVisit, false);
    }

    public PurePursuitCommand(PIDFController xController, PIDFController yController, RotationPIDFController rotationController,
                              List<Point> pointsToVisit, boolean disableFastDrive) {
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;
        this.pointsToVisit = pointsToVisit;
        this.disableFastDrive = disableFastDrive;

        addRequirements(robotDrive, robotOdometry);
    }

    @Override
    public void execute() {
        Point targetPoint = pointsToVisit.get(pointIndex);

        if (targetPoint == null) {
            return;
        }

        Pose2d targetPose = computeTargetPose(targetPoint);

        ChassisSpeeds computedSpeed = computeSpeed(targetPose, targetPoint.motionBudget());

        Log.i("PurePursuit", String.valueOf(computedSpeed));

        robotDrive.drive(computedSpeed, true, new Rotation2d());

        displayOnField();
    }

    @Override
    public boolean isFinished() {
        if (pointsToVisit == null || pointsToVisit.isEmpty()) {
            return true;
        }

        return (pointIndex == pointsToVisit.size() - 1) && xController.atSetPoint() && yController.atSetPoint() && rotationController.atSetPoint();
    }

    private void displayOnField() {
        TelemetryPacket fieldPacket = GlobalSubsystem.getInstance().fieldPacket;

        fieldPacket.fieldOverlay().setTranslation(0, 0).setRotation(0).setStrokeWidth(1);

        Pose2d robotPose = robotOdometry.getPose();
        Point targetPoint = pointsToVisit.get(pointIndex);

        if (targetPoint != null) {
            fieldPacket.fieldOverlay().setStroke("yellow").strokeLine(robotPose.getY(), -robotPose.getX(), targetPoint.pose().getY(), -targetPoint.pose().getX());
        }

        for (Point point : pointsToVisit) {
            Pose2d pose = point.pose();
            Translation2d ahead = RobotMath.getTranslationOffPoint(pose.getTranslation(), 12, pose.getHeading());
            Translation2d behind = RobotMath.getTranslationOffPoint(pose.getTranslation(), -12, pose.getHeading());

            fieldPacket.fieldOverlay().setStroke("blue").strokeLine(ahead.getY(), -ahead.getX(), pose.getY(), -pose.getX());
            fieldPacket.fieldOverlay().setStroke("red").strokeLine(behind.getY(), -behind.getX(), pose.getY(), -pose.getX());
        }
    }

    private Pose2d computeTargetPose(Point targetPoint) {
        Pose2d robotPose = robotOdometry.getPose();

        Pose2d targetPose = targetPoint.pose();

        double distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());

        if (Math.abs(distanceToTarget) < advancePointThreshold) {
            Log.i("aaaaaaa", "distance is " + distanceToTarget + ", advancing");
            // Switch to the next point once in range
            pointIndex++;

            if (pointIndex >= pointsToVisit.size()) {
                pointIndex = pointsToVisit.size() - 1;
            }
        }

        Translation2d targetTranslation = targetPose.getTranslation();
        Rotation2d targetRotation = targetPose.getRotation();

        if (Math.abs(distanceToTarget) > lookAtTargetThreshold && !disableFastDrive) {
            // Look at the target to go faster when we're further away
            targetRotation = new Rotation2d(RobotMath.getAngleToPoint(robotPose.getTranslation(), targetTranslation));
            rotationController.setContinuous(true);
        } else if (targetPoint.translationOnly() && !disableFastDrive) {
            if (pointIndex + 1 <= pointsToVisit.size() - 1) {
                // Start trying to look towards the next point
                Pose2d nextPose = pointsToVisit.get(pointIndex + 1).pose();
                targetRotation = new Rotation2d(RobotMath.getAngleToPoint(robotPose.getTranslation(), nextPose.getTranslation()));
            }
            // Keep the rotation controller continuous since rotation doesn't matter for this point
            rotationController.setContinuous(true);
        } else {
            rotationController.setContinuous(false);
        }

        return new Pose2d(targetTranslation, targetRotation);
    }

    private ChassisSpeeds computeSpeed(Pose2d targetPose, double motionBudget) {
        Pose2d robotPose = robotOdometry.getPose();
        Pose2d robotPose2 = robotDrive.getPose();
        double xPower = xController.calculate(robotPose.getX(), targetPose.getX());
        double yPower = yController.calculate(robotPose.getY(), targetPose.getY());
        double rotPower = rotationController.calculate(robotPose2.getRotation(), targetPose.getRotation());
        rotPower = Math.copySign(Math.min(Math.abs(rotPower), motionBudget), rotPower);

        final double remainingMotionBudget = motionBudget - Math.abs(rotPower);

        // Scale the output
        if (Math.abs(xPower) > remainingMotionBudget || Math.abs(yPower) > remainingMotionBudget) {
            if (Math.abs(xPower) > Math.abs(yPower)) {
                double scale = remainingMotionBudget / Math.abs(xPower);
                xPower = MathUtils.clamp(xPower, -remainingMotionBudget, remainingMotionBudget);
                yPower = yPower * scale;
            } else {
                double scale = remainingMotionBudget / Math.abs(yPower);
                xPower = xPower * scale;
                yPower = MathUtils.clamp(yPower, -remainingMotionBudget, remainingMotionBudget);
            }
        }

        return new ChassisSpeeds(xPower, yPower, rotPower);
    }
}
