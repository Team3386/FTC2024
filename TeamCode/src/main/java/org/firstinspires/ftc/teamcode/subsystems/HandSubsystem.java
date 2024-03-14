package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.HandConstants;


public class HandSubsystem extends SubsystemBase {
    private static final HandSubsystem INSTANCE = new HandSubsystem();

    private ServoEx leftServo, rightServo;

    private Telemetry telemetry;

    private boolean leftState, rightState;

    private HandSubsystem() {
    }

    public static HandSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;
        telemetry = GlobalSubsystem.getInstance().telemetry;

        leftServo = new SimpleServo(hardwareMap, HandConstants.LEFT_SERVO_NAME, 0, 0);
        rightServo = new SimpleServo(hardwareMap, HandConstants.RIGHT_SERVO_NAME, 0, 0);

        rightServo.setInverted(true);
    }

    public void toggleLeftState() {
        leftState = !leftState;
    }

    public void toggleRightState() {
        rightState = !rightState;
    }

    public void setLeftState(boolean state) {
        leftState = state;
    }

    public void setRightState(boolean state) {
        rightState = state;
    }

    public void drive(double power) {
//        final double mod = power;
        //position open right 0.2
        //position closed right 0.5
        //position open left 0.9
        //position closed left 0.3
//        if (mod != 0) {
//            if (Math.round(rightServo.getPosition() * 100d) / 100d == HandConstants.RIGHT_FULLY_OPEN_POSITION) {
//                rightServo.setPosition(HandConstants.RIGHT_CLOSED_ON_PIXEL_POSITION);
//            } else {
//                rightServo.setPosition(HandConstants.RIGHT_FULLY_OPEN_POSITION);
//            }
//
//            if (Math.round(leftServo.getPosition() * 100d) / 100d == HandConstants.LEFT_FULLY_OPEN_POSITION) {
//                leftServo.setPosition(HandConstants.LEFT_CLOSED_ON_PIXEL_POSITION);
//            } else {
//                leftServo.setPosition(HandConstants.LEFT_FULLY_OPEN_POSITION);
//            }
//        }
    }

    @Override
    public void periodic() {
        telemetry.addData("left angle", leftServo.getAngle());
        telemetry.addData("right angle", rightServo.getAngle());
        telemetry.addData("left position", leftServo.getPosition());
        telemetry.addData("right position", rightServo.getPosition());
        telemetry.addData("leftState", leftState);
        telemetry.addData("rightState", rightState);
        if (leftState) {
            leftServo.setPosition(HandConstants.LEFT_OPEN_POSITION);
        } else {
            leftServo.setPosition(0.45);
        }

        if (rightState) {
            rightServo.setPosition(HandConstants.RIGHT_OPEN_POSITION);
        } else {
            rightServo.setPosition(0.7);
        }
    }
}
