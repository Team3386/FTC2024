package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;


public class PlaneSubsystem extends SubsystemBase {
    private static final PlaneSubsystem INSTANCE = new PlaneSubsystem();

    private ServoEx planeServo;

    private Telemetry telemetry;

    private PlaneSubsystem() {
    }

    public static PlaneSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;
        telemetry = GlobalSubsystem.getInstance().telemetry;

        planeServo = new SimpleServo(hardwareMap, Constants.PlaneConstants.PLANE_SERVO_NAME, 60, 100);
    }

    public void drive(double power) {
        if (power > 0) {
            planeServo.rotateByAngle(power * 5);
        } else {
            planeServo.setPosition(0.7);
        }
    }

    @Override
    public void periodic() {
        telemetry.addData("Plane: Position", planeServo.getPosition());
    }
}
