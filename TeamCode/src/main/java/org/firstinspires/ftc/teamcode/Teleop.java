package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private static final Robot robot = Robot.getInstance();

    @Override
    public void init() {

        robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.teleopInit();
    }

    @Override
    public void loop() {
        robot.teleopPeriodic();
    }
}
