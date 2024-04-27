package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.AbstractMotorSubsystem;
import org.firstinspires.ftc.teamcode.utils.AnalogEncoder;

public class ArmExtendSubsystem extends AbstractMotorSubsystem {
    private static final ArmExtendSubsystem INSTANCE = new ArmExtendSubsystem();
    private MotorEx motor;

    // NOTE: Limit switches are inverted
    private DigitalChannel topLimit, bottomLimit;

    private AnalogEncoder encoder;

    private Telemetry telemetry;

    private ArmExtendSubsystem() {
    }

    public static ArmExtendSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();

        telemetry = globalSubsystem.telemetry;

        motor = new MotorEx(globalSubsystem.hardwareMap, Constants.ArmConstants.EXTEND_MOTOR_NAME);

        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        motor.setInverted(true);
        encoder = new AnalogEncoder(Constants.ArmConstants.EXTEND_ENCODER_NAME, 360);

        topLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, Constants.ArmConstants.TOP_LIMIT_SWITCH_NAME);
        bottomLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, Constants.ArmConstants.BOTTOM_LIMIT_SWITCH_NAME);

        topLimit.setMode(DigitalChannel.Mode.INPUT);
        bottomLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    public double get() {
        return encoder.getRotation();
    }

    public void setMotor(double power) {
        telemetry.addData("Arm: Extend power", power);
        if ((!topLimit.getState() && power < 0) || (!bottomLimit.getState() && power > 0)) {
            motor.set(0);
        } else {
            motor.set(power);
        }
    }

    @Override
    public void periodic() {
        telemetry.addData("Arm: Extend limit top", topLimit.getState());
        telemetry.addData("Arm: Extend limit bottom", bottomLimit.getState());
        telemetry.addData("Arm: Extend", encoder.getRotation());
    }
}
