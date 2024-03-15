package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.utils.AnalogEncoder;

public class ArmSubsystem extends SubsystemBase {
    private static final ArmSubsystem INSTANCE = new ArmSubsystem();
    private MotorEx rotationMotor;
    private MotorEx extendMotor;
    private MotorEx wristMotor;
    private AnalogEncoder rotationEncoder, extendEncoder;

    private DistanceSensor sonarSensor;

    // NOTE: Limit switches are inverted
    private DigitalChannel topLimit, bottomLimit;

    private Telemetry telemetry;

    private ArmSubsystem() {
    }

    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();

        telemetry = globalSubsystem.telemetry;

        rotationMotor = new MotorEx(globalSubsystem.hardwareMap, ArmConstants.ROTATION_MOTOR_NAME);
        extendMotor = new MotorEx(globalSubsystem.hardwareMap, ArmConstants.EXTEND_MOTOR_NAME);
        wristMotor = new MotorEx(globalSubsystem.hardwareMap, ArmConstants.WRIST_MOTOR_NAME);

        rotationMotor.setRunMode(RunMode.RawPower);
        rotationMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        extendMotor.setRunMode(RunMode.RawPower);
        extendMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        wristMotor.setRunMode(RunMode.RawPower);
        wristMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        rotationMotor.setInverted(true);
        extendMotor.setInverted(true);

        wristMotor.resetEncoder();

        rotationEncoder = new AnalogEncoder(ArmConstants.ROTATION_ENCODER_NAME, 270);
        extendEncoder = new AnalogEncoder(ArmConstants.EXTEND_ENCODER_NAME, 360);

        topLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, ArmConstants.TOP_LIMIT_SWITCH_NAME);
        bottomLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, ArmConstants.BOTTOM_LIMIT_SWITCH_NAME);

        topLimit.setMode(DigitalChannel.Mode.INPUT);
        bottomLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    public double getRotation() {
        return rotationEncoder.getRotation();
    }

    public void setRotationMotor(double power) {
        if ((rotationEncoder.getRotation() <= 0.09 && power < 0) || (rotationEncoder.getRotation() >= 250 && power > 0)) {
            rotationMotor.set(0);
        } else {
            rotationMotor.set(power);
        }
    }

    public void stopRotation() {
        rotationMotor.set(0);
    }

    public double getWrist() {
        return wristMotor.getCurrentPosition();
    }

    public void setWristMotor(double power) {
        wristMotor.set(power);
    }

    public void stopWrist() {
        wristMotor.set(0);
    }

    public double getExtend() {
        return extendEncoder.getRotation();
    }

    public void setExtendMotor(double power) {
        if ((topLimit.getState() && power < 0) || (bottomLimit.getState() && power > 0)) {
            extendMotor.set(power);
        }
    }

    public void stopExtend() {
        extendMotor.set(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("Arm: Rotation", rotationEncoder.getRotation());
        telemetry.addData("Arm: Wrist", wristMotor.getCurrentPosition());
        telemetry.addData("Arm: Extend limit top", topLimit.getState());
        telemetry.addData("Arm: Extend limit bottom", bottomLimit.getState());
        telemetry.addData("Arm: Extend", extendEncoder.getRotation());
    }
}
