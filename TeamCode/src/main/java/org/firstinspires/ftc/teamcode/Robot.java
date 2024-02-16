package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HandSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneSubsystem;

public class Robot {
    private static final Robot INSTANCE = new Robot();

    private static final GlobalSubsystem robotGlobal = GlobalSubsystem.getInstance();
    private static final DriveSubsystem robotDrive = DriveSubsystem.getInstance();
    //    private static final VisionSubsystem robotVision = VisionSubsystem.getInstance();
    private static final ArmSubsystem robotArm = ArmSubsystem.getInstance();
    private static final HandSubsystem robotHand = HandSubsystem.getInstance();

    private static final PlaneSubsystem robotPlane = PlaneSubsystem.getInstance();

    private static GamepadEx pilotController;
    private static GamepadEx copilotController;

    private Robot() {
    }

    public static Robot getInstance() {
        return INSTANCE;
    }


    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        pilotController = new GamepadEx(gamepad1);
        copilotController = new GamepadEx(gamepad2);

        robotGlobal.init(telemetry, hardwareMap);
        robotDrive.init();
        robotArm.init();
        robotHand.init();
        robotPlane.init();
//        robotVision.init();

        telemetry.addData("Status", "Initialized");
    }

    public void teleopInit() {
        robotDrive.setDefaultCommand(new RunCommand(() -> {
//            double slowdownFactor = 1 - pilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.75;
//            slowdownFactor *= 1 - pilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.75;

            final double movementSlowdown = 1 - pilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.85;
            final double rotationSlowdown = 1 - pilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.85;
            robotDrive.drive(new ChassisSpeeds(
                    pilotController.getLeftX() * movementSlowdown * Constants.DriveConstants.MAX_MOVEMENT_PER_SECOND,
                    pilotController.getLeftY() * movementSlowdown * Constants.DriveConstants.MAX_MOVEMENT_PER_SECOND,
                    pilotController.getRightX() * rotationSlowdown * Constants.DriveConstants.MAX_ROTATION_PER_SECOND
            ), true);

            if (pilotController.wasJustPressed(GamepadKeys.Button.START)) {
                robotDrive.resetGyro();
            }
        }, robotDrive));

        robotArm.setDefaultCommand(new RunCommand(() -> {
            if (copilotController.isDown(GamepadKeys.Button.BACK)) {
                robotArm.resetBottom();
            }

            robotArm.rotateArm(
                    (copilotController.isDown(GamepadKeys.Button.DPAD_UP) ? 1 : 0) +
                            (copilotController.isDown(GamepadKeys.Button.DPAD_DOWN) ? -1 : 0)
            );
            robotArm.rotateWrist(
                    (copilotController.isDown(GamepadKeys.Button.A) ? 1 : 0) +
                            (copilotController.isDown(GamepadKeys.Button.X) ? -1 : 0)
            );
            robotArm.extendArm(
                    (copilotController.isDown(GamepadKeys.Button.DPAD_RIGHT) ? 1 : 0) +
                            (copilotController.isDown(GamepadKeys.Button.DPAD_LEFT) ? -1 : 0)
            );

            if (copilotController.isDown(GamepadKeys.Button.B)) {
                robotArm.setWristTarget(ArmConstants.WRIST_PARALLEL_TO_GROUND);
            }

            if (pilotController.isDown(GamepadKeys.Button.X)) {
                robotArm.setOverride(ArmConstants.ROTATION_PASS_DEGREE, ArmConstants.WRIST_HIGH_DEGREE);
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

    public void teleopPeriodic() {
        pilotController.readButtons();
        copilotController.readButtons();
        periodic();
    }

    public void periodic() {
        CommandScheduler.getInstance().run();
    }
}
