package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class Teleop extends Robot {
    @Override
    public void start() {
        robotDrive.setDefaultCommand(new RunCommand(() -> {
            double movementSlowdown = 1 - pilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.6;
            movementSlowdown *= 1 - pilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.4;

            robotDrive.drive(new ChassisSpeeds(
                    pilotController.getLeftX() * movementSlowdown * Constants.DriveConstants.MAX_MOVEMENT_PER_SECOND,
                    pilotController.getLeftY() * movementSlowdown * Constants.DriveConstants.MAX_MOVEMENT_PER_SECOND,
                    pilotController.getRightX() * movementSlowdown * Constants.DriveConstants.MAX_ROTATION_PER_SECOND
            ), true, robotGlobal.driverRotation);

            if (pilotController.wasJustPressed(GamepadKeys.Button.START)) {
                robotDrive.resetPose(new Pose2d().rotate(-Math.PI * 1.5));
            }
        }, robotDrive));

        robotOdometry.setDefaultCommand(new RunCommand(() -> {
            if (pilotController.wasJustPressed(GamepadKeys.Button.START)) {
                robotOdometry.resetPose(new Pose2d().rotate(-Math.PI * 1.5));
            }
        }, robotOdometry));

        robotArm.setDefaultCommand(new RunCommand(() -> {
            if (copilotController.isDown(GamepadKeys.Button.DPAD_UP) || copilotController.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                double power = 1 + copilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 2;
                robotArm.setRotationMotor(
                        (copilotController.isDown(GamepadKeys.Button.DPAD_UP) ? power : 0) +
                                (copilotController.isDown(GamepadKeys.Button.DPAD_DOWN) ? -power : 0)
                );
            } else if (copilotController.wasJustReleased(GamepadKeys.Button.DPAD_UP) || copilotController.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                robotArm.stopRotation();
            }

            if (copilotController.isDown(GamepadKeys.Button.X) || copilotController.isDown(GamepadKeys.Button.A)) {
                robotArm.setWristMotor(
                        (copilotController.isDown(GamepadKeys.Button.X) ? -0.3 : 0) +
                                (copilotController.isDown(GamepadKeys.Button.A) ? 0.3 : 0)
                );
            } else if (copilotController.wasJustReleased(GamepadKeys.Button.X) || copilotController.wasJustReleased(GamepadKeys.Button.A)) {
                robotArm.stopWrist();
            }

            if (copilotController.isDown(GamepadKeys.Button.DPAD_LEFT) || copilotController.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                double power = 0.5 + copilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.5;
                robotArm.setExtendMotor(

                        (copilotController.isDown(GamepadKeys.Button.DPAD_RIGHT) ? -power : 0) +
                                (copilotController.isDown(GamepadKeys.Button.DPAD_LEFT) ? power : 0)
                );
            } else if (copilotController.wasJustReleased(GamepadKeys.Button.DPAD_LEFT) || copilotController.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
                robotArm.stopExtend();
            }
        }, robotArm));

        robotHand.setDefaultCommand(new RunCommand(() -> {
            robotHand.setLeftState(copilotController.isDown(GamepadKeys.Button.LEFT_BUMPER));
            robotHand.setRightState(copilotController.isDown(GamepadKeys.Button.RIGHT_BUMPER));
        }, robotHand));

        robotPlane.setDefaultCommand(new RunCommand(() -> robotPlane.drive(
                (copilotController.isDown(GamepadKeys.Button.Y) ? 1 : 0)
        ), robotPlane));
    }
}
