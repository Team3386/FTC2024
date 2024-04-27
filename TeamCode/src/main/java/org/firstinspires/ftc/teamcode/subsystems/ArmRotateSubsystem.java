package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.AbstractMotorSubsystem;
import org.firstinspires.ftc.teamcode.utils.AnalogEncoder;

public class ArmRotateSubsystem extends AbstractMotorSubsystem {
    private static final ArmRotateSubsystem INSTANCE = new ArmRotateSubsystem();

    private MotorEx motor;
    private AnalogEncoder encoder;
    private Telemetry telemetry;

    private ArmRotateSubsystem() {
    }

    public static ArmRotateSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();

        telemetry = globalSubsystem.telemetry;

        motor = new MotorEx(globalSubsystem.hardwareMap, Constants.ArmConstants.ROTATION_MOTOR_NAME);

        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        motor.setInverted(true);

        encoder = new AnalogEncoder(Constants.ArmConstants.ROTATION_ENCODER_NAME, 270);
    }

    public double get() {
        return encoder.getRotation();
    }

    public void setMotor(double power) {
        if ((encoder.getRotation() <= 0.09 && power < 0) || (encoder.getRotation() >= 250 && power > 0)) {
            motor.set(0);
        } else {
            motor.set(power);
        }
    }

    @Override
    public void periodic() {
        telemetry.addData("Arm: Rotation", encoder.getRotation());
    }
}
