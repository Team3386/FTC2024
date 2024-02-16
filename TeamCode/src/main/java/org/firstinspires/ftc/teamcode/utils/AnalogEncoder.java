package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;

public class AnalogEncoder {
    private final AnalogInput sensor;
    private final double totalRotation;

    public AnalogEncoder(String name, double totalRotation) {
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;

        sensor = hardwareMap.get(AnalogInput.class, name);
        this.totalRotation = totalRotation;
    }

    public double getRotation() {
        return sensor.getVoltage() / RobotConstants.ANALOG_NOMINAL_VOLTAGE * totalRotation;
    }
}
