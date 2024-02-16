package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;
import org.firstinspires.ftc.teamcode.utils.AnalogEncoder;

@TeleOp(name = "encodertest")
public class encodertest extends OpMode {
    private static final GlobalSubsystem robotGlobal = GlobalSubsystem.getInstance();
//    private static final VisionSubsystem robotVision = VisionSubsystem.getInstance();

    private static GamepadEx pilotController;
    private static GamepadEx copilotController;

    AnalogEncoder analogEncoder;

    @Override
    public void init() {
        pilotController = new GamepadEx(gamepad1);
        copilotController = new GamepadEx(gamepad2);

        robotGlobal.init(telemetry, hardwareMap);

        analogEncoder = new AnalogEncoder("a", 360);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.update();
    }
}
