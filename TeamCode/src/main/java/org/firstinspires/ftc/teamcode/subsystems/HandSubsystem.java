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

        leftState = false;
        rightState = false;
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

    @Override
    public void periodic() {
        telemetry.addData("Hand: Left", leftServo.getPosition());
        telemetry.addData("Hand: Right", rightServo.getPosition());
        telemetry.addData("Hand: State left", leftState);
        telemetry.addData("Hand: State right", rightState);
        if (leftState) {
            leftServo.setPosition(HandConstants.LEFT_OPEN_POSITION);
        } else {
            leftServo.setPosition(0.6);
        }

        if (rightState) {
            rightServo.setPosition(HandConstants.RIGHT_OPEN_POSITION);
        } else {
            rightServo.setPosition(0.75);
        }
    }
}
