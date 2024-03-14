package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.AutonomousConstants;
import org.firstinspires.ftc.teamcode.commands.purepursuit.Point;
import org.firstinspires.ftc.teamcode.commands.purepursuit.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoBlueBack extends Robot {
    public void init() {
        super.init();
        DriveSubsystem.getInstance().resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        OdometrySubsystem.getInstance().resetPose(new Pose2d(new Translation2d(-182.88 + (45f / 2f), -91.44), Rotation2d.fromDegrees(90)));
    }

    public void start() {
        robotVision.fixProp();

        PIDFController rotation = new PIDFController(AutonomousConstants.ARM_ROTATION_PID[0], AutonomousConstants.ARM_ROTATION_PID[1], AutonomousConstants.ARM_ROTATION_PID[2], 0.);
        PIDFController wrist = new PIDFController(AutonomousConstants.ARM_WRIST_PID[0], AutonomousConstants.ARM_WRIST_PID[1], AutonomousConstants.ARM_WRIST_PID[2], 0.);
        PIDFController extend = new PIDFController(AutonomousConstants.ARM_EXTEND_PID[0], AutonomousConstants.ARM_EXTEND_PID[1], AutonomousConstants.ARM_EXTEND_PID[2], 0.);

        List<Point> path1 = new ArrayList<>();

        path1.add(new Point(new Translation2d(-91.44, -91.44)));


        List<Point> path2 = new ArrayList<>();

        path2.add(new Point(new Translation2d(-91.44, 91.44)));

        List<Point> path3 = new ArrayList<>();
        path3.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));

        List<Point> path4 = new ArrayList<>();
        path4.add(new Point(new Translation2d(-91.44, 91.44)));

        List<Point> path5 = new ArrayList<>();
        path5.add(new Point(new Translation2d(-91.44, -91.44)));

        List<Point> path6 = new ArrayList<>();
        path6.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));

        List<Point> points = new ArrayList<>();
        if (robotVision.propPos() != 0) {
            points.add(new Point(new Translation2d(-91.44, -91.44)));
            points.add(new Point(new Translation2d(-91.44, 91.44)));
            points.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));
            points.add(new Point(new Translation2d(-91.44, -91.44)));
            points.add(new Point(new Pose2d(new Translation2d(-80, -137.16), Rotation2d.fromDegrees(180)), 75));
            points.add(new Point(new Translation2d(-30.48, -91.44)));
            points.add(new Point(new Translation2d(-30.48, 30.48)));
            points.add(new Point(new Pose2d(new Translation2d(-91.44, 91.44), Rotation2d.fromDegrees(0))));
            points.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));
            points.add(new Point(new Translation2d(-91.44, -91.44)));
        } else {
            points.add(new Point(new Translation2d(-30.48, -91.44)));
            points.add(new Point(new Translation2d(-30.48, 30.48)));
            points.add(new Point(new Pose2d(new Translation2d(-91.44, 91.44), Rotation2d.fromDegrees(0))));
            points.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));
            points.add(new Point(new Translation2d(-152.4, 91.44)));
            points.add(new Point(new Translation2d(-152.4, -91.44)));
            points.add(new Point(new Pose2d(new Translation2d(-80, -137.16), Rotation2d.fromDegrees(180)), 75));
            points.add(new Point(new Translation2d(-30.48, -91.44)));
            points.add(new Point(new Translation2d(-30.48, 30.48)));
            points.add(new Point(new Pose2d(new Translation2d(-91.44, 91.44), Rotation2d.fromDegrees(0))));
            points.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));
            points.add(new Point(new Translation2d(-152.4, 91.44)));
            points.add(new Point(new Translation2d(-152.4, -91.44)));
        }

//        CommandScheduler.getInstance().schedule(
//                // Go to entrance with arm lowered
//                new PurePursuitCommand(path1).alongWith(new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArm::getRotation, robotArm::setRotationMotor, robotArm))
//                        .andThen(
//                                // Drive through
//                                new PurePursuitCommand(path2)
//                        )
//                        .andThen(
//                                // Drive to backboard with arm raised
//                                new PurePursuitCommand(path3)
//                                        .alongWith(new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArm::getRotation, robotArm::setRotationMotor, robotArm))
//                        )
//                        .andThen(
//                                // Drive back to entrance with arm lowered
//                                new PurePursuitCommand(path4)
//                                        .alongWith(new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArm::getRotation, robotArm::setRotationMotor, robotArm))
//                        )
//                        .andThen(
//                                // Drive through
//                                new PurePursuitCommand(path5)
//                        )
//                        .andThen(
//                                // Drive to pickup and lower arm to pickup position
//                                new PurePursuitCommand(path6)
//                                        .alongWith(new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PICKUP, robotArm::getRotation, robotArm::setRotationMotor, robotArm))
//                        )
//        );
        CommandScheduler.getInstance().schedule(
                new PurePursuitCommand(points)
        );
    }
}
