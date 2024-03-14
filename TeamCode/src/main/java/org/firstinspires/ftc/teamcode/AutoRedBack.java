package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.utils.RotationPIDFController;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoRedBack extends Robot {
    public void init() {
        super.init();
        DriveSubsystem.getInstance().resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(-90)));
        OdometrySubsystem.getInstance().resetPose(new Pose2d(new Translation2d(152.4, -91.44), Rotation2d.fromDegrees(-90)));
    }

    public void start() {
        List<Point> points = new ArrayList<>();

        //SequentialCommandGroup

        points.add(new Point(new Translation2d(91.44, -91.44)));
        points.add(new Point(new Translation2d(91.44, 91.44)));
        points.add(new Point(new Pose2d(new Translation2d(91.44, 91.44), Rotation2d.fromDegrees(0))));
        points.add(new Point(new Pose2d(new Translation2d(91.44, 110), Rotation2d.fromDegrees(0)), 75));
        points.add(new Point(new Translation2d(91.44, -91.44)));
        points.add(new Point(new Translation2d(80, -100)));
        points.add(new Point(new Pose2d(new Translation2d(80, -137.16), Rotation2d.fromDegrees(180)), 75));
        points.add(new Point(new Translation2d(30.48, -91.44)));
        points.add(new Point(new Translation2d(30.48, 30.48)));
        points.add(new Point(new Pose2d(new Translation2d(91.44, 91.44), Rotation2d.fromDegrees(0))));
        points.add(new Point(new Pose2d(new Translation2d(91.44, 110), Rotation2d.fromDegrees(0)), 75));
        points.add(new Point(new Translation2d(91.44, -91.44)));
//        points.add(new Point(new Pose2d(new Translation2d(-152.4, -91.44), Rotation2d.fromDegrees(90))));
//        points.add(new Point(new Pose2d(new Translation2d(75, 0), Rotation2d.fromDegrees(0)), true));
//        points.add(new Point(new Pose2d(new Translation2d(0, 75), Rotation2d.fromDegrees(0)), true));
//        points.add(new Point(new Pose2d(new Translation2d(20, 45), Rotation2d.fromDegrees(67)), false));
//        points.add(new Point(new Pose2d(new Translation2d(75, 75), Rotation2d.fromDegrees(0)), true));
//        points.add(new Point(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90)), false));

        PurePursuitCommand command = new PurePursuitCommand(
                new PIDFController(
                        AutonomousConstants.POSITION_PID[0],
                        AutonomousConstants.POSITION_PID[1],
                        AutonomousConstants.POSITION_PID[2],
                        0
                ),
                new PIDFController(
                        AutonomousConstants.POSITION_PID[0],
                        AutonomousConstants.POSITION_PID[1],
                        AutonomousConstants.POSITION_PID[2],
                        0
                ),
                new RotationPIDFController(
                        AutonomousConstants.ROTATION_PID[0],
                        AutonomousConstants.ROTATION_PID[1],
                        AutonomousConstants.ROTATION_PID[2],
                        0
                ),
                points
        );

        //CommandScheduler.getInstance().schedule(command.whenFinished(() -> {
        //    SelectCommand
        //}));
    }
}
