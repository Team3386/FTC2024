package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp(name = "OpenCV Vision Test")
public class VisionTest extends OpMode {
    private static final GlobalSubsystem robotGlobal = GlobalSubsystem.getInstance();
    private static final VisionSubsystem robotVision = VisionSubsystem.getInstance();

    @Override
    public void init() {
        robotGlobal.init(telemetry, hardwareMap);
        robotVision.init();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
