package org.firstinspires.ftc.teamcode.commands.purepursuit;

import static org.firstinspires.ftc.teamcode.Constants.PI_2;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
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
    private final DriveSubsystem robotDrive = DriveSubsystem.getInstance();
    private final OdometrySubsystem robotOdometry = OdometrySubsystem.getInstance();

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
    private boolean commandDidFinish = true;
    private long lastLog;

    public PurePursuitCommand(List<Point> pointsToVisit) {
        this(
                new PIDFController(
                        Constants.AutonomousConstants.X_PID[0],
                        Constants.AutonomousConstants.X_PID[1],
                        Constants.AutonomousConstants.X_PID[2],
                        0
                ),
                new PIDFController(
                        Constants.AutonomousConstants.Y_PID[0],
                        Constants.AutonomousConstants.Y_PID[1],
                        Constants.AutonomousConstants.Y_PID[2],
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
    public void initialize() {
        if (pointsToVisit == null || pointsToVisit.isEmpty()) {
            return;
        }
        Point point = pointsToVisit.get(0);

        final CommandScheduler commandScheduler = CommandScheduler.getInstance();
        Command pointCommand = point.command();
        if (pointCommand != null) {
            commandScheduler.schedule(pointCommand);
        }
        commandDidFinish = !point.waitForCommand();
        commandScheduler.onCommandFinish(this::finishCallback);
    }

    @Override
    public void execute() {
        Point targetPoint;

        try {
            targetPoint = pointsToVisit.get(pointIndex);
        } catch (Exception e) {
            return;
        }

        Pose2d targetPose = computeTargetPose(targetPoint);

        ChassisSpeeds computedSpeed = computeSpeed(targetPose, targetPoint.motionBudget());

        if (System.nanoTime() - lastLog > 1e+9) {
            Log.i("PurePursuit", String.valueOf(computedSpeed));
            Log.i("PurePursuit", String.format("Current point: %d/%d", pointIndex, pointsToVisit.size() - 1));
            final double distance = Math.abs(targetPoint.pose().getTranslation().getDistance(robotOdometry.getPose().getTranslation()));
            Log.i("PurePursuit", String.format("Distance to target: %f", distance));
            lastLog = System.nanoTime();
        }

        if (!robotDrive.autoOverride) {
            robotDrive.drive(computedSpeed, true, new Rotation2d(), 1.25);
        }

        displayOnField();
    }

    @Override
    public boolean isFinished() {
        if (pointsToVisit == null || pointsToVisit.isEmpty()) {
            return true;
        }

        Pose2d robotPose = robotOdometry.getPose();

        Pose2d targetPose = pointsToVisit.get(pointIndex).pose();

        double distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());

        return (pointIndex == pointsToVisit.size() - 1) && ((xController.atSetPoint() && yController.atSetPoint() && rotationController.atSetPoint()) || distanceToTarget < advancePointThreshold * 0.75);
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


        final double distanceToTarget = Math.abs(robotPose.getTranslation().getDistance(targetPose.getTranslation()));

        boolean rotationAligned = true;
        if (!targetPoint.translationOnly()) {
            Rotation2d rotationError = targetPose.getRotation().minus(robotPose.getRotation());
            if (Math.abs(rotationError.getRadians()) > PI_2) {
                rotationError = new Rotation2d(rotationError.getRadians() - Math.copySign(Math.PI, rotationError.getRadians()));
            }
            if (Math.abs(rotationError.getDegrees()) > 5) {
                rotationAligned = false;
            }
        }

        if (distanceToTarget < advancePointThreshold && rotationAligned && commandDidFinish && pointIndex != pointsToVisit.size() - 1) {
            Log.i("PurePursuit", String.format("Distance to target: %f < %f, advancing point...", distanceToTarget, advancePointThreshold));
            // Switch to the next point once in range
            pointIndex++;

            if (pointIndex >= pointsToVisit.size()) {
                pointIndex = pointsToVisit.size() - 1;
            } else {
                try {
                    Point nextPoint = pointsToVisit.get(pointIndex);
                    Command commandToSchedule = nextPoint.command();
                    if (commandToSchedule != null) {
                        Log.i("PurePursuit", "Point has command, scheduling...");
                        CommandScheduler.getInstance().schedule(commandToSchedule);
                        commandDidFinish = !nextPoint.waitForCommand();
                    } else {
                        commandDidFinish = true;
                    }
                } catch (Exception ignored) {
                }
            }
        }

        Translation2d targetTranslation = targetPose.getTranslation();
        Rotation2d targetRotation = targetPose.getRotation();

        if (distanceToTarget > lookAtTargetThreshold && !disableFastDrive) {
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

    public void finishCallback(Command command) {
        Point point = pointsToVisit.get(pointIndex);
        if (command == point.command()) {
            commandDidFinish = true;
        }
    }
}
