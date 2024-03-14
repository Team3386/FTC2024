package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HandSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public abstract class Robot extends OpMode {
    static final GlobalSubsystem robotGlobal = GlobalSubsystem.getInstance();
    static final DriveSubsystem robotDrive = DriveSubsystem.getInstance();
    static final OdometrySubsystem robotOdometry = OdometrySubsystem.getInstance();
    static final VisionSubsystem robotVision = VisionSubsystem.getInstance();
    static final ArmSubsystem robotArm = ArmSubsystem.getInstance();
    static final HandSubsystem robotHand = HandSubsystem.getInstance();
    static final PlaneSubsystem robotPlane = PlaneSubsystem.getInstance();

    static GamepadEx pilotController;
    static GamepadEx copilotController;

    public void init() {
        pilotController = new GamepadEx(gamepad1);
        copilotController = new GamepadEx(gamepad2);

        robotGlobal.init(telemetry, hardwareMap);
        robotDrive.init();
        robotOdometry.init();
        robotVision.init();
        robotArm.init();
        robotHand.init();
        robotPlane.init();

        robotGlobal.telemetry.addData("Status", "Initialized");

        CommandScheduler.getInstance().registerSubsystem(robotGlobal, robotDrive, robotOdometry, robotVision, robotArm, robotHand, robotPlane);
    }

    public void init_loop() {
        pilotController.readButtons();
        copilotController.readButtons();
        CommandScheduler.getInstance().run();
    }

    public void loop() {
        final double start = System.nanoTime();
        pilotController.readButtons();
        copilotController.readButtons();
        CommandScheduler.getInstance().run();
        robotGlobal.telemetry.addData("Loop time", (System.nanoTime() - start) / 1e6);
    }

    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}
