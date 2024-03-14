package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RevIMU extends GyroEx {

    private final IntegratingGyroscope IMU;
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
    private AxesOrder axesOrder = AxesOrder.ZXY;
    private int multiplier;

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub
     *
     * @param hw      Hardware map
     * @param imuName Name of sensor in configuration
     */
    public RevIMU(HardwareMap hw, String imuName) {
        IMU = hw.get(IntegratingGyroscope.class, imuName);
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
        return IMU.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.RADIANS).firstAngle * multiplier;
    }

    /**
     * @return X, Y, Z angles of gyro
     */
    public double[] getAngles() {
        // make a singular hardware call
        Orientation orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.RADIANS);

        return new double[]{
                orientation.firstAngle,
                orientation.secondAngle,
                orientation.thirdAngle
        };
    }

    public void setAxesOrder(AxesOrder newOrder) {
        axesOrder = newOrder;
    }

    /**
     * @return Transforms heading into {@link Rotation2d}
     */
    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(getHeading());
    }

    @Override
    public void disable() {
    }

    @Override
    public void reset() {
        offset += getHeading();
    }

    @Override
    public String getDeviceType() {
        return "IMU";
    }

    /**
     * @return the internal sensor being wrapped
     */
    public IntegratingGyroscope getIMU() {
        return IMU;
    }

}
