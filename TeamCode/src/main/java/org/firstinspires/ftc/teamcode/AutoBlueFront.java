//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Translation2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Constants.AutonomousConstants;
//import org.firstinspires.ftc.teamcode.commands.ToTargetCommand;
//import org.firstinspires.ftc.teamcode.commands.WristToTargetCommand;
//import org.firstinspires.ftc.teamcode.commands.purepursuit.Point;
//import org.firstinspires.ftc.teamcode.commands.purepursuit.PurePursuitCommand;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Autonomous
//public class AutoBlueFront extends Robot {
//    @Override
//    public void init() {
//        super.init();
//        robotDrive.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
//        robotOdometry.resetPose(new Pose2d(new Translation2d(-182.88 + (45f / 2f), 30.48), Rotation2d.fromDegrees(90)));
//        robotGlobal.driverRotation = Rotation2d.fromDegrees(90);
//        robotVision.propPos = 2;
//    }
//
//    @Override
//    public void start() {
//        PIDFController wrist = new PIDFController(AutonomousConstants.ARM_WRIST_PID[0], AutonomousConstants.ARM_WRIST_PID[1], AutonomousConstants.ARM_WRIST_PID[2], 0.);
//
//        wrist.setTolerance(10);
//
//        CommandScheduler.getInstance().schedule(
//                new WristToTargetCommand(wrist, AutonomousConstants.ARM_WRIST_UP, robotArm::getWrist, robotArm::setWristMotor).alongWith(new WaitCommand(AutonomousConstants.WAIT_BEFORE_LOCK)).andThen(
//                        new InstantCommand(() -> {
//                            robotVision.lockProp();
//                            CommandScheduler.getInstance().schedule(generatePath(robotVision.propPos));
//                        })
//                )
//        );
//    }
//
//    public PurePursuitCommand generatePath(int prop) {
//        PIDFController rotation = new PIDFController(AutonomousConstants.ARM_ROTATION_PID[0], AutonomousConstants.ARM_ROTATION_PID[1], AutonomousConstants.ARM_ROTATION_PID[2], 0.);
//        PIDFController wrist = new PIDFController(AutonomousConstants.ARM_WRIST_PID[0], AutonomousConstants.ARM_WRIST_PID[1], AutonomousConstants.ARM_WRIST_PID[2], 0.);
//        PIDFController extend = new PIDFController(AutonomousConstants.ARM_EXTEND_PID[0], AutonomousConstants.ARM_EXTEND_PID[1], AutonomousConstants.ARM_EXTEND_PID[2], 0.);
//
//        rotation.setTolerance(3);
//        wrist.setTolerance(10);
//        extend.setTolerance(3);
//
//        List<Point> points = new ArrayList<>();
//
//        switch (prop) {
//            case 0:
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, 60.96),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new ToTargetCommand(
//                                rotation, AutonomousConstants.ARM_ROTATION_PICKUP,
//                                robotArm::getRotation, robotArm::setRotationMotor
//                        ).alongWith(
//                                new ToTargetCommand(
//                                        extend, AutonomousConstants.ARM_EXTEND_PICKUP,
//                                        robotArm::getExtend, robotArm::setExtendMotor
//                                ),
//                                new WristToTargetCommand(
//                                        wrist, AutonomousConstants.ARM_WRIST_PICKUP,
//                                        robotArm::getWrist, robotArm::setWristMotor
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-115, 60.96),
//                                Rotation2d.fromDegrees(90)
//                        ))
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-115, 60.96),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new InstantCommand(() -> robotHand.setRightOpen(true), robotHand).alongWith(
//                                new WaitCommand(AutonomousConstants.WAIT_BEFORE_UP).andThen(
//                                        new WristToTargetCommand(
//                                                wrist, AutonomousConstants.ARM_WRIST_UP,
//                                                robotArm::getWrist, robotArm::setWristMotor),
//                                        new InstantCommand(() ->
//                                                CommandScheduler.getInstance().schedule(
//                                                        new ToTargetCommand(
//                                                                rotation, AutonomousConstants.ARM_ROTATION_PASS,
//                                                                robotArm::getRotation, robotArm::setRotationMotor
//                                                        ),
//                                                        new ToTargetCommand(
//                                                                extend, AutonomousConstants.ARM_EXTEND_TOP,
//                                                                robotArm::getExtend, robotArm::setExtendMotor
//                                                        )
//                                                )
//                                        )
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, 30.48),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        75,
//                        new WristToTargetCommand(
//                                wrist, AutonomousConstants.ARM_WRIST_DOWN,
//                                robotArm::getWrist, robotArm::setWristMotor
//                        ).alongWith(
//                                new InstantCommand(() -> robotHand.setRightOpen(false), robotHand)
//                        ),
//                        false)
//                );
//                break;
//            case 1:
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, 30.48),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new ToTargetCommand(
//                                rotation, AutonomousConstants.ARM_ROTATION_PICKUP,
//                                robotArm::getRotation, robotArm::setRotationMotor
//                        ).alongWith(
//                                new ToTargetCommand(
//                                        extend, AutonomousConstants.ARM_EXTEND_PICKUP,
//                                        robotArm::getExtend, robotArm::setExtendMotor
//                                ),
//                                new WristToTargetCommand(
//                                        wrist, AutonomousConstants.ARM_WRIST_PICKUP,
//                                        robotArm::getWrist, robotArm::setWristMotor
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-98.25, 30.48),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new WaitCommand(200), true
//                ));
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-98.25, 30.48),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new InstantCommand(() -> {
//                            robotHand.setRightOpen(true);
//                        }, robotHand).alongWith(
//                                new WaitCommand(AutonomousConstants.WAIT_BEFORE_UP).andThen(
//                                        new WristToTargetCommand(
//                                                wrist, AutonomousConstants.ARM_WRIST_UP,
//                                                robotArm::getWrist, robotArm::setWristMotor),
//                                        new InstantCommand(() ->
//                                                CommandScheduler.getInstance().schedule(
//                                                        new ToTargetCommand(
//                                                                rotation, AutonomousConstants.ARM_ROTATION_PASS,
//                                                                robotArm::getRotation, robotArm::setRotationMotor
//                                                        ),
//                                                        new ToTargetCommand(
//                                                                extend, AutonomousConstants.ARM_EXTEND_TOP,
//                                                                robotArm::getExtend, robotArm::setExtendMotor
//                                                        )
//                                                )
//                                        )
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, 30.48),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        75,
//                        new WristToTargetCommand(
//                                wrist, AutonomousConstants.ARM_WRIST_DOWN,
//                                robotArm::getWrist, robotArm::setWristMotor
//                        ).alongWith(
//                                new InstantCommand(() -> robotHand.setRightOpen(false), robotHand)
//                        ),
//                        false)
//                );
//                break;
//            case 2:
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-91.44, 60.96),
//                                Rotation2d.fromDegrees(180)
//                        ),
//                        new ToTargetCommand(
//                                rotation, AutonomousConstants.ARM_ROTATION_PICKUP,
//                                robotArm::getRotation, robotArm::setRotationMotor
//                        ).alongWith(
//                                new ToTargetCommand(
//                                        extend, AutonomousConstants.ARM_EXTEND_PICKUP,
//                                        robotArm::getExtend, robotArm::setExtendMotor
//                                ),
//                                new WristToTargetCommand(
//                                        wrist, AutonomousConstants.ARM_WRIST_PICKUP,
//                                        robotArm::getWrist, robotArm::setWristMotor
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-91.44, 35),
//                                Rotation2d.fromDegrees(180)
//                        ),
//                        new WaitCommand(200), true
//                ));
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-91.44, 35),
//                                Rotation2d.fromDegrees(180)
//                        ),
//                        new InstantCommand(() -> {
//                            robotHand.setRightOpen(true);
//                        }, robotHand).alongWith(
//                                new WaitCommand(AutonomousConstants.WAIT_BEFORE_UP).andThen(
//                                        new WristToTargetCommand(
//                                                wrist, AutonomousConstants.ARM_WRIST_UP,
//                                                robotArm::getWrist, robotArm::setWristMotor),
//                                        new InstantCommand(() ->
//                                                CommandScheduler.getInstance().schedule(
//                                                        new ToTargetCommand(
//                                                                rotation, AutonomousConstants.ARM_ROTATION_PASS,
//                                                                robotArm::getRotation, robotArm::setRotationMotor
//                                                        ),
//                                                        new ToTargetCommand(
//                                                                extend, AutonomousConstants.ARM_EXTEND_TOP,
//                                                                robotArm::getExtend, robotArm::setExtendMotor
//                                                        )
//                                                )
//                                        )
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(new Translation2d(-91.44, 76.2)));
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, 30.48),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        75,
//                        new WristToTargetCommand(
//                                wrist, AutonomousConstants.ARM_WRIST_DOWN,
//                                robotArm::getWrist, robotArm::setWristMotor
//                        ).alongWith(
//                                new InstantCommand(() -> robotHand.setRightOpen(false), robotHand)
//                        ),
//                        false)
//                );
//                break;
//        }
//
//        points.add(new Point(new Translation2d(-152.4, 152.4)));
//        points.add(new Point(new Translation2d(-152.4, 152.4), new WaitCommand(500).andThen(new InstantCommand(() -> robotHand.setLeftOpen(true), robotHand)), false));
//
////        switch (prop) {
////            case 0:
////                points.add(new Point(new Translation2d(-152.4, -152.4)));
////                points.add(new Point(new Translation2d(-30.48, -152.4)));
////                break;
////            case 1:
////                points.add(new Point(new Translation2d(-91.44, -91.44),
////                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArm::getRotation, robotArm::setRotationMotor), true));
////                points.add(new Point(new Translation2d(-91.44, 30.48)));
////                points.add(new Point(new Translation2d(-91.44, 91.44),
////                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArm::getRotation, robotArm::setRotationMotor), false));
////                points.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));
////                points.add(new Point(new Translation2d(-91.44, 30.48),
////                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArm::getRotation, robotArm::setRotationMotor), true));
////                points.add(new Point(new Translation2d(-91.44, -91.44)));
////                points.add(new Point(new Pose2d(new Translation2d(-80, -137.16), Rotation2d.fromDegrees(180)), 75));
////                points.add(new Point(new Translation2d(-30.48, -91.44)));
////                points.add(new Point(new Translation2d(-30.48, 30.48)));
////                points.add(new Point(new Translation2d(-91.44, 91.44),
////                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArm::getRotation, robotArm::setRotationMotor), false));
////                points.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));
////                points.add(new Point(new Translation2d(-91.44, 30.48),
////                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArm::getRotation, robotArm::setRotationMotor), true));
////                points.add(new Point(new Translation2d(-91.44, -91.44)));
////        }
//
//        return new PurePursuitCommand(points);
//    }
//}
