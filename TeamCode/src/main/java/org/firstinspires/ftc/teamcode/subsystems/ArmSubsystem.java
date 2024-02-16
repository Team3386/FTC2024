package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.utils.AnalogEncoder;

public class ArmSubsystem extends SubsystemBase {
    private static final ArmSubsystem INSTANCE = new ArmSubsystem();
    private final PIDFController rotationController = new PIDFController(1, 0, 0, 0);
    private final PIDFController wristController = new PIDFController(0.015, 0, 0, 0);

    private final PIDFController extendController = new PIDFController(1, 0, 0, 0);
    private MotorEx rotationMotor;
    private MotorEx extendMotor;
    private MotorEx wristMotor;
    private AnalogEncoder rotationEncoder;

    // NOTE: Limit switches are inversed
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
        extendMotor.setRunMode(RunMode.RawPower);
        wristMotor.setRunMode(RunMode.RawPower);

        rotationMotor.setInverted(true);

        rotationEncoder = new AnalogEncoder(ArmConstants.ROTATION_ENCODER_NAME, 360 * 10);

        topLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, ArmConstants.TOP_LIMIT_SWITCH_NAME);
        bottomLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, ArmConstants.BOTTOM_LIMIT_SWITCH_NAME);

        topLimit.setMode(DigitalChannel.Mode.INPUT);
        bottomLimit.setMode(DigitalChannel.Mode.INPUT);

        rotationController.setSetPoint(rotationEncoder.getRotation());
        rotationController.setTolerance(5);

        wristController.setSetPoint(wristMotor.getCurrentPosition());
        wristController.setTolerance(5);

    }

    public void rotateArm(double power) {
        final double currentRotation = rotationEncoder.getRotation();
        if (power == 0 && !rotationController.atSetPoint()) {
            rotationController.setSetPoint(MathUtils.clamp(
                    currentRotation,
                    ArmConstants.ROTATION_LOW_DEGREE,
                    ArmConstants.ROTATION_HIGH_DEGREE
            ));
        }
        rotationController.setSetPoint(MathUtils.clamp(
                MathUtils.clamp(
                        rotationController.getSetPoint() + power * 2,
                        currentRotation - 20,
                        currentRotation + 20
                ),
                ArmConstants.ROTATION_LOW_DEGREE,
                ArmConstants.ROTATION_HIGH_DEGREE
        ));
    }

    public void rotateWrist(double power) {
        wristController.setSetPoint(MathUtils.clamp(
                wristController.getSetPoint() + power * 2,
                ArmConstants.WRIST_LOW_DEGREE,
                ArmConstants.WRIST_HIGH_DEGREE
        ));
    }

    public void setWristTarget(double target) {
        wristController.setSetPoint(MathUtils.clamp(
                target,
                ArmConstants.WRIST_LOW_DEGREE,
                ArmConstants.WRIST_HIGH_DEGREE
        ));
    }

    public void extendArm(double power) {
        if (topLimit.getState() && power > 0) {
            extendMotor.set(power);
        }
        if (bottomLimit.getState() && power < 0) {
            extendMotor.set(power);
        }
        if (power == 0) {
            extendMotor.set(0);
        }
    }


    @Override
    public void periodic() {
        final double rotationPower = rotationController.calculate(rotationEncoder.getRotation());
        final double wristPower = wristController.calculate(wristMotor.getCurrentPosition());
        telemetry.addData("Arm rotation", rotationEncoder.getRotation());
        telemetry.addData("Arm rotation target", rotationController.getSetPoint());
        telemetry.addData("Arm rotation at point", rotationController.atSetPoint());
        telemetry.addData("Calculated rotation force", rotationPower);
        telemetry.addData("Wrist position", wristMotor.getCurrentPosition());
        telemetry.addData("Wrist position target", wristController.getSetPoint());
        telemetry.addData("Wrist position at point", wristController.atSetPoint());
        telemetry.addData("Calculated wrist force", wristPower);
        telemetry.addData("Top", topLimit.getState());
        telemetry.addData("Bottom", bottomLimit.getState());
        telemetry.addData("Extend motor Speed", extendMotor.getVelocity());
        telemetry.addData("Extend motor position", extendMotor.getCurrentPosition());
        if (!rotationController.atSetPoint()) {
            rotationMotor.set(rotationPower);
        } else {
            rotationMotor.set(0);
        }

        if (!wristController.atSetPoint() && Math.abs(wristPower) > 0.25) {
            wristMotor.set(wristPower);
        } else {
            wristMotor.set(0);
        }
    }
}
