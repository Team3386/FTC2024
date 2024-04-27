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

    private boolean leftOpen, rightOpen;

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

        leftOpen = false;
        rightOpen = false;
    }

    public void toggleLeft() {
        leftOpen = !leftOpen;
    }

    public void toggleRight() {
        rightOpen = !rightOpen;
    }

    public void setLeftOpen(boolean state) {
        leftOpen = state;
    }

    public void setRightOpen(boolean state) {
        rightOpen = state;
    }

    @Override
    public void periodic() {
        telemetry.addData("Hand: Left", leftServo.getPosition());
        telemetry.addData("Hand: Right", rightServo.getPosition());
        telemetry.addData("Hand: Open left", leftOpen);
        telemetry.addData("Hand: Open right", rightOpen);
        if (leftOpen) {
            leftServo.setPosition(HandConstants.LEFT_OPEN_POSITION);
        } else {
            leftServo.setPosition(0.60);
        }

        if (rightOpen) {
            rightServo.setPosition(HandConstants.RIGHT_OPEN_POSITION);
        } else {
            rightServo.setPosition(0.80);
        }
    }
}
