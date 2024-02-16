package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RevIMU extends GyroEx {

    private final IMU revIMU;
    /***
     * Heading relative to starting position
     */
    double globalHeading;
    /**
     * Heading relative to last offset
     */
    double relativeHeading;
    /**
     * Offset between global heading and relative heading
     */
    double offset;
    private int multiplier;

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub
     *
     * @param hw      Hardware map
     * @param imuName Name of sensor in configuration
     */
    public RevIMU(HardwareMap hw, String imuName) {
        revIMU = hw.get(IMU.class, imuName);
        multiplier = 1;
    }

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub with the default configuration name of "imu"
     *
     * @param hw Hardware map
     */
    public RevIMU(HardwareMap hw) {
        this(hw, "imu");
    }

    /**
     * Initializes gyro with custom parameters.
     */
    public void init(IMU.Parameters parameters) {
        revIMU.initialize(parameters);

        globalHeading = 0;
        relativeHeading = 0;
        offset = 0;
    }

    /**
     * Inverts the ouptut of gyro
     */
    public void invertGyro() {
        multiplier *= -1;
    }

    @Override
    public void init() {
    }

    /**
     * @return Relative heading of the robot
     */
    public double getHeading() {
        // Return yaw
        return getAbsoluteHeading() - offset;
    }

    /**
     * @return Absolute heading of the robot
     */
    @Override
    public double getAbsoluteHeading() {
        return revIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) * multiplier;
    }

    /**
     * @return X, Y, Z angles of gyro
     */
    public double[] getAngles() {
        // make a singular hardware call
        YawPitchRollAngles yawPitchRoll = revIMU.getRobotYawPitchRollAngles();

        return new double[]{
                yawPitchRoll.getYaw(AngleUnit.DEGREES),
                yawPitchRoll.getPitch(AngleUnit.DEGREES),
                yawPitchRoll.getRoll(AngleUnit.DEGREES)
        };
    }

    /**
     * @return Transforms heading into {@link Rotation2d}
     */
    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void disable() {
        revIMU.close();
    }

    @Override
    public void reset() {
        offset += getHeading();
    }

    @Override
    public String getDeviceType() {
        return "Rev Expansion Hub IMU";
    }

    /**
     * @return the internal sensor being wrapped
     */
    public IMU getRevIMU() {
        return revIMU;
    }

}
