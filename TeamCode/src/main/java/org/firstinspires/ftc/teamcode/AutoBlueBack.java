package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants.AutonomousConstants;
import org.firstinspires.ftc.teamcode.commands.AlignWithApriltag;
import org.firstinspires.ftc.teamcode.commands.ToTargetCommand;
import org.firstinspires.ftc.teamcode.commands.WristToTargetCommand;
import org.firstinspires.ftc.teamcode.commands.purepursuit.Point;
import org.firstinspires.ftc.teamcode.commands.purepursuit.PurePursuitCommand;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoBlueBack extends Robot {
    @Override
    public void init() {
        super.init();
        robotDrive.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        robotOdometry.resetPose(new Pose2d(new Translation2d(-182.88 + (45f / 2f), -91.44), Rotation2d.fromDegrees(90)));
        robotGlobal.driverRotation = Rotation2d.fromDegrees(90);
        robotVision.propPos = 0;
        robotArmWrist.resetEncoder();
    }

    @Override
    public void start() {
        PIDFController wrist = new PIDFController(AutonomousConstants.ARM_WRIST_PID[0], AutonomousConstants.ARM_WRIST_PID[1], AutonomousConstants.ARM_WRIST_PID[2], 0.);

        wrist.setTolerance(10);

        CommandScheduler.getInstance().schedule(
                new WristToTargetCommand(wrist, AutonomousConstants.ARM_WRIST_UP).alongWith(new WaitCommand(AutonomousConstants.WAIT_BEFORE_LOCK)).andThen(
                        new InstantCommand(() -> {
                            robotVision.lockProp();
                            CommandScheduler.getInstance().schedule(generatePath(robotVision.propPos));
                        })
                )
        );
    }

    public PurePursuitCommand generatePath(int prop) {
        PIDFController rotation = new PIDFController(AutonomousConstants.ARM_ROTATION_PID[0], AutonomousConstants.ARM_ROTATION_PID[1], AutonomousConstants.ARM_ROTATION_PID[2], 0.);
        PIDFController wrist = new PIDFController(AutonomousConstants.ARM_WRIST_PID[0], AutonomousConstants.ARM_WRIST_PID[1], AutonomousConstants.ARM_WRIST_PID[2], 0.);
        PIDFController extend = new PIDFController(AutonomousConstants.ARM_EXTEND_PID[0], AutonomousConstants.ARM_EXTEND_PID[1], AutonomousConstants.ARM_EXTEND_PID[2], 0.);

        rotation.setTolerance(3);
        wrist.setTolerance(15);
        extend.setTolerance(3);

        List<Point> points = new ArrayList<>();

//        switch (prop) {
//            case 0:
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-91.44, -121.92),
//                                Rotation2d.fromDegrees(0)
//                        ),
//                        new ToTargetCommand(
//                                rotation, AutonomousConstants.ARM_ROTATION_PICKUP,
//                                robotArmRotate::get, robotArmRotate::setMotor
//                        ).alongWith(
//                                new ToTargetCommand(
//                                        extend, AutonomousConstants.ARM_EXTEND_PICKUP,
//                                        robotArmExtend::get, robotArmExtend::setMotor
//                                ),
//                                new WristToTargetCommand(
//                                        wrist, AutonomousConstants.ARM_WRIST_UP,
//                                        robotArmWrist::get, robotArmWrist::setMotor
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-91.44, -99),
//                                Rotation2d.fromDegrees(0)
//                        ),
//                        new WaitCommand(200), true
//                ));
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-91.44, -99),
//                                Rotation2d.fromDegrees(0)
//                        ),
//                        new InstantCommand(() -> {
//                            robotHand.setRightOpen(true);
//                            CommandScheduler.getInstance().schedule(
//                                    new ToTargetCommand(
//                                            extend, AutonomousConstants.ARM_EXTEND_TOP,
//                                            robotArmExtend::get, robotArmExtend::setMotor
//                                    )
//                            );
//                        }, robotHand).alongWith(
//                                new WaitCommand(AutonomousConstants.WAIT_BEFORE_UP)
//                        ),
//                        true)
//                );
//                points.add(new Point(new Translation2d(-91.44, -121.92),
//                        new WristToTargetCommand(
//                                wrist, AutonomousConstants.ARM_WRIST_UP,
//                                robotArmWrist::get, robotArmWrist::setMotor).alongWith(new ToTargetCommand(
//                                        rotation, AutonomousConstants.ARM_ROTATION_PASS,
//                                        robotArmRotate::get, robotArmRotate::setMotor
//                                )
//                        ), false));
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, -91.44),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        75,
//                        new WristToTargetCommand(
//                                wrist, AutonomousConstants.ARM_WRIST_DOWN,
//                                robotArmWrist::get, robotArmWrist::setMotor
//                        ).alongWith(
//                                new InstantCommand(() -> robotHand.setRightOpen(false), robotHand)
//                        ),
//                        true)
//                );
//                break;
//            case 1:
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, -91.44),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new ToTargetCommand(
//                                rotation, AutonomousConstants.ARM_ROTATION_PICKUP,
//                                robotArmRotate::get, robotArmRotate::setMotor
//                        ).alongWith(
//                                new ToTargetCommand(
//                                        extend, AutonomousConstants.ARM_EXTEND_PICKUP,
//                                        robotArmExtend::get, robotArmExtend::setMotor
//                                ),
//                                new WristToTargetCommand(
//                                        wrist, AutonomousConstants.ARM_WRIST_PICKUP,
//                                        robotArmWrist::get, robotArmWrist::setMotor
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-99, -91.44),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new WaitCommand(200), true
//                ));
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-99, -91.44),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new InstantCommand(() -> {
//                            robotHand.setRightOpen(true);
//                        }, robotHand).alongWith(
//                                new WaitCommand(AutonomousConstants.WAIT_BEFORE_UP).andThen(
//                                        new WristToTargetCommand(
//                                                wrist, AutonomousConstants.ARM_WRIST_UP,
//                                                robotArmWrist::get, robotArmWrist::setMotor),
//                                        new InstantCommand(() ->
//                                                CommandScheduler.getInstance().schedule(
//                                                        new ToTargetCommand(
//                                                                rotation, AutonomousConstants.ARM_ROTATION_PASS,
//                                                                robotArmRotate::get, robotArmRotate::setMotor
//                                                        ),
//                                                        new ToTargetCommand(
//                                                                extend, AutonomousConstants.ARM_EXTEND_TOP,
//                                                                robotArmExtend::get, robotArmExtend::setMotor
//                                                        )
//                                                )
//                                        )
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, -91.44),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        75,
//                        new WristToTargetCommand(
//                                wrist, AutonomousConstants.ARM_WRIST_DOWN,
//                                robotArmWrist::get, robotArmWrist::setMotor
//                        ).alongWith(
//                                new InstantCommand(() -> robotHand.setRightOpen(false), robotHand)
//                        ),
//                        false)
//                );
//                break;
//            case 2:
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, -112.5),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new ToTargetCommand(
//                                rotation, AutonomousConstants.ARM_ROTATION_PICKUP,
//                                robotArmRotate::get, robotArmRotate::setMotor
//                        ).alongWith(
//                                new ToTargetCommand(
//                                        extend, AutonomousConstants.ARM_EXTEND_PICKUP,
//                                        robotArmExtend::get, robotArmExtend::setMotor
//                                ),
//                                new WristToTargetCommand(
//                                        wrist, AutonomousConstants.ARM_WRIST_UP,
//                                        robotArmWrist::get, robotArmWrist::setMotor
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-115, -115),
//                                Rotation2d.fromDegrees(90)
//                        ))
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-115, -115),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        new InstantCommand(() -> robotHand.setRightOpen(true), robotHand).alongWith(
//                                new WaitCommand(AutonomousConstants.WAIT_BEFORE_UP).andThen(
//                                        new WristToTargetCommand(
//                                                wrist, AutonomousConstants.ARM_WRIST_UP,
//                                                robotArmWrist::get, robotArmWrist::setMotor),
//                                        new InstantCommand(() ->
//                                                CommandScheduler.getInstance().schedule(
//                                                        new ToTargetCommand(
//                                                                rotation, AutonomousConstants.ARM_ROTATION_PASS,
//                                                                robotArmRotate::get, robotArmRotate::setMotor
//                                                        ),
//                                                        new ToTargetCommand(
//                                                                extend, AutonomousConstants.ARM_EXTEND_TOP,
//                                                                robotArmExtend::get, robotArmExtend::setMotor
//                                                        )
//                                                )
//                                        )
//                                )
//                        ),
//                        true)
//                );
//                points.add(new Point(
//                        new Pose2d(
//                                new Translation2d(-152.4, -91.44),
//                                Rotation2d.fromDegrees(90)
//                        ),
//                        75,
//                        new WristToTargetCommand(
//                                wrist, AutonomousConstants.ARM_WRIST_DOWN,
//                                robotArmWrist::get, robotArmWrist::setMotor
//                        ).alongWith(
//                                new InstantCommand(() -> robotHand.setRightOpen(false), robotHand)
//                        ),
//                        false)
//                );
//                break;
//        }

//        switch (prop) {
//            case 0:
//                points.add(new Point(new Translation2d(-152.4, -152.4)));
//                points.add(new Point(new Translation2d(-30.48, -152.4)));
//                break;
//            case 1:
//                points.add(new Point(new Translation2d(-91.44, -91.44),
//                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArmRotate::get, robotArmRotate::setMotor), true));
//                points.add(new Point(new Translation2d(-91.44, 30.48)));
//                points.add(new Point(new Translation2d(-91.44, 91.44),
//                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArmRotate::get, robotArmRotate::setMotor), false));
//                points.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));
//                points.add(new Point(new Translation2d(-91.44, 30.48),
//                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArmRotate::get, robotArmRotate::setMotor), true));
//                points.add(new Point(new Translation2d(-91.44, -91.44)));
//                points.add(new Point(new Pose2d(new Translation2d(-80, -137.16), Rotation2d.fromDegrees(180)), 75));
//                points.add(new Point(new Translation2d(-30.48, -91.44)));
//                points.add(new Point(new Translation2d(-30.48, 30.48)));
//                points.add(new Point(new Translation2d(-91.44, 91.44),
//                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArmRotate::get, robotArmRotate::setMotor), false));
//                points.add(new Point(new Pose2d(new Translation2d(-91.44, 110), Rotation2d.fromDegrees(0)), 75));
//                points.add(new Point(new Translation2d(-91.44, 30.48),
//                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArmRotate::get, robotArmRotate::setMotor), true));
//                points.add(new Point(new Translation2d(-91.44, -91.44)));
//        }

        points.add(new Point(new Translation2d(-91.44, -91.44),
                new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArmRotate).alongWith(
                        new WristToTargetCommand(
                                wrist, AutonomousConstants.ARM_WRIST_DOWN
                        )
                ), true));
        points.add(new Point(new Translation2d(-91.44, 30.48)));
        points.add(new Point(new Translation2d(-91.44, 91.44),
                new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArmRotate).alongWith(
                        new ToTargetCommand(
                                extend, AutonomousConstants.ARM_EXTEND_BACKBOARD,
                                robotArmExtend
                        ),
                        new WristToTargetCommand(
                                wrist, AutonomousConstants.ARM_WRIST_BACKBOARD
                        )
                ), false));
        points.add(new Point(new Pose2d(new Translation2d(-91.44, 127), Rotation2d.fromDegrees(0)), 75,
                new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArmRotate).alongWith(
                        new ToTargetCommand(
                                extend, AutonomousConstants.ARM_EXTEND_BACKBOARD,
                                robotArmExtend
                        ),
                        new WristToTargetCommand(
                                wrist, AutonomousConstants.ARM_WRIST_BACKBOARD
                        )
                ), true));
        points.add(new Point(new Pose2d(new Translation2d(-91.44, 127), Rotation2d.fromDegrees(0)), new AlignWithApriltag(2), true));
        points.add(new Point(new Pose2d(new Translation2d(-91.44, 127), Rotation2d.fromDegrees(0)),
                new InstantCommand(() -> robotHand.setLeftOpen(true)).andThen(new WaitCommand(AutonomousConstants.WAIT_BEFORE_UP)),
                true)
        );
        points.add(new Point(new Pose2d(new Translation2d(-91.44, 91.44), Rotation2d.fromDegrees(0)), 75));
        points.add(new Point(new Translation2d(-91.44, 30.48),
                new InstantCommand(() -> {
                    robotHand.setLeftOpen(false);
                    CommandScheduler.getInstance().schedule(
                            new ToTargetCommand(
                                    extend, AutonomousConstants.ARM_EXTEND_PICKUP_STACK,
                                    robotArmExtend
                            ),
                            new ToTargetCommand(
                                    wrist, AutonomousConstants.ARM_WRIST_DOWN,
                                    robotArmWrist
                            )
                    );
                }).alongWith(
                        new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PASS, robotArmRotate)
                ),
                true)
        );
        points.add(new Point(new Translation2d(-91.44, -100), new ToTargetCommand(
                extend, AutonomousConstants.ARM_EXTEND_PICKUP_STACK, robotArmExtend
        ).alongWith(
                new InstantCommand(() -> robotHand.setRightOpen(true)),
                new WristToTargetCommand(
                        wrist, AutonomousConstants.ARM_WRIST_DOWN
                ),
                new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArmRotate)
        ), true));

        points.add(new Point(new Pose2d(new Translation2d(-85, -115), Rotation2d.fromDegrees(180)), new WristToTargetCommand(
                wrist, AutonomousConstants.ARM_WRIST_PICKUP
        ), true));
        points.add(new Point(new Pose2d(new Translation2d(-85, -115), Rotation2d.fromDegrees(180)), new AlignWithApriltag(9), true));
        points.add(new Point(new Pose2d(new Translation2d(-85, -115), Rotation2d.fromDegrees(180)), new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_PICKUP, robotArmRotate), true));


        points.add(new Point(new Pose2d(new Translation2d(-85, -141), Rotation2d.fromDegrees(180)), 75));

        points.add(new Point(new Pose2d(new Translation2d(-85, -141), Rotation2d.fromDegrees(180)),
                new InstantCommand(() -> robotHand.setRightOpen(false))
                        .alongWith(new WaitCommand(AutonomousConstants.WAIT_BEFORE_UP)),
                true)
        );

        points.add(new Point(new Translation2d(-91.44, -91.44)));

        points.add(new Point(new Translation2d(-91.44, 30.48)));
        points.add(new Point(new Translation2d(-91.44, 91.44),
                new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArmRotate).alongWith(
                        new ToTargetCommand(
                                extend, AutonomousConstants.ARM_EXTEND_BACKBOARD,
                                robotArmExtend
                        ),
                        new WristToTargetCommand(
                                wrist, AutonomousConstants.ARM_WRIST_BACKBOARD
                        )
                ), false));
        points.add(new Point(new Pose2d(new Translation2d(-91.44, 100), Rotation2d.fromDegrees(0)), 75,
                new ToTargetCommand(rotation, AutonomousConstants.ARM_ROTATION_BACKBOARD, robotArmRotate).alongWith(
                        new ToTargetCommand(
                                extend, AutonomousConstants.ARM_EXTEND_BACKBOARD,
                                robotArmExtend
                        ),
                        new WristToTargetCommand(
                                wrist, AutonomousConstants.ARM_WRIST_BACKBOARD
                        )
                ), true));
        points.add(new Point(new Pose2d(new Translation2d(-91.44, 105), Rotation2d.fromDegrees(0)), new AlignWithApriltag(2), true));
        points.add(new Point(new Pose2d(new Translation2d(-91.44, 127), Rotation2d.fromDegrees(0))));
        points.add(new Point(new Pose2d(new Translation2d(-91.44, 127), Rotation2d.fromDegrees(0)),
                new InstantCommand(() -> robotHand.setRightOpen(true)).andThen(new WaitCommand(AutonomousConstants.WAIT_BEFORE_UP)),
                true)
        );

        return new PurePursuitCommand(points);
    }
}
