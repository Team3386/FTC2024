package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.AbstractMotorSubsystem;

public class ArmWristSubsystem extends AbstractMotorSubsystem {
    private static final ArmWristSubsystem INSTANCE = new ArmWristSubsystem();

    private MotorEx motor;
    private Telemetry telemetry;

    private ArmWristSubsystem() {
    }

    public static ArmWristSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();

        telemetry = globalSubsystem.telemetry;

        motor = new MotorEx(globalSubsystem.hardwareMap, Constants.ArmConstants.WRIST_MOTOR_NAME);

        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }

    public double get() {
        return motor.getCurrentPosition();
    }

    public void setMotor(double power) {
        motor.set(power);
    }

    public void resetEncoder() {
        motor.resetEncoder();
    }

    @Override
    public void periodic() {
        telemetry.addData("Arm: Wrist", motor.getCurrentPosition());
    }
}
