package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public class GlobalSubsystem extends SubsystemBase {
    private static final GlobalSubsystem INSTANCE = new GlobalSubsystem();
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket fieldPacket = new TelemetryPacket();
    public ElapsedTime elapsedTime = new ElapsedTime();
    private List<LynxModule> allHubs;

    private GlobalSubsystem() {
    }

    public static GlobalSubsystem getInstance() {
        return INSTANCE;
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        this.hardwareMap = hardwareMap;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void periodic() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        dashboard.sendTelemetryPacket(fieldPacket);
        telemetry.update();
        fieldPacket = new TelemetryPacket();
        fieldPacket.fieldOverlay()
                .setScale(Constants.CENTIMETER_PER_INCH_INVERSE, Constants.CENTIMETER_PER_INCH_INVERSE);
    }
}
