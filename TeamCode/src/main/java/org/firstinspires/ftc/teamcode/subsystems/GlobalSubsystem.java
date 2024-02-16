package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class GlobalSubsystem extends SubsystemBase {
    private static final GlobalSubsystem INSTANCE = new GlobalSubsystem();
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public ElapsedTime elapsedTime = new ElapsedTime();

    private GlobalSubsystem() {
    }

    public static GlobalSubsystem getInstance() {
        return INSTANCE;
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void periodic() {
        telemetry.update();
    }
}
