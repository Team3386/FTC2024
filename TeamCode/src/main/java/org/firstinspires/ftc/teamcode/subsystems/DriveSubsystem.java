package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.utils.RevIMU;

public class DriveSubsystem extends SubsystemBase {
    private static final DriveSubsystem INSTANCE = new DriveSubsystem();
    private static final MecanumDriveOdometry odometry = new MecanumDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d());
    private static MotorEx frontLeft;
    private static MotorEx frontRight;
    private static MotorEx rearLeft;
    private static MotorEx rearRight;
    private static RevIMU imu;

    private static ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

    private DriveSubsystem() {
    }

    public static DriveSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();

        frontLeft = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.FRONT_LEFT_MOTOR_NAME);
        frontRight = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        rearLeft = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.REAR_LEFT_MOTOR_NAME);
        rearRight = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.REAR_RIGHT_MOTOR_NAME);

        frontLeft.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        frontRight.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        rearLeft.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        rearRight.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);

        frontLeft.setInverted(true);
        rearRight.setInverted(true);

        frontLeft.setRunMode(RunMode.VelocityControl);
        frontRight.setRunMode(RunMode.VelocityControl);
        rearLeft.setRunMode(RunMode.VelocityControl);
        rearRight.setRunMode(RunMode.VelocityControl);

        imu = new RevIMU(globalSubsystem.hardwareMap);
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        targetChassisSpeeds = fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                        chassisSpeeds.omegaRadiansPerSecond, imu.getRotation2d())
                : chassisSpeeds;
    }

    @Override
    public void periodic() {
        Telemetry telemetry = GlobalSubsystem.getInstance().telemetry;

        telemetry.addData("Target Chassis X", targetChassisSpeeds.vxMetersPerSecond);
        telemetry.addData("Target Chassis Y", targetChassisSpeeds.vyMetersPerSecond);
        telemetry.addData("Target Chassis Rad", targetChassisSpeeds.omegaRadiansPerSecond);

        targetChassisSpeeds.vxMetersPerSecond = -targetChassisSpeeds.vxMetersPerSecond;

        MecanumDriveWheelSpeeds wheelSpeeds = DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(targetChassisSpeeds);

        telemetry.addData("frontleft", frontLeft.getVelocity());
        telemetry.addData("frontright", frontRight.getVelocity());
        telemetry.addData("rearleft", rearLeft.getVelocity());
        telemetry.addData("rearright", rearRight.getVelocity());
        telemetry.addData("frontleft", frontLeft.getDistance());
        telemetry.addData("frontright", frontRight.getDistance());
        telemetry.addData("rearleft", rearLeft.getDistance());
        telemetry.addData("rearright", rearRight.getDistance());
        telemetry.addData("a", frontLeft.getCPR());

        telemetry.addData("IMU data", imu.getRotation2d());
        telemetry.addData("IMU raw orientation", imu.getRevIMU().getRobotYawPitchRollAngles());

        telemetry.addData("Calculated wheel speeds", wheelSpeeds);

        frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * DriveConstants.DISTANCE_TO_TICKS_INVERSE);
        frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond * DriveConstants.DISTANCE_TO_TICKS_INVERSE);
        rearLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * DriveConstants.DISTANCE_TO_TICKS_INVERSE);
        rearRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond * DriveConstants.DISTANCE_TO_TICKS_INVERSE);

        if (targetChassisSpeeds.vxMetersPerSecond != 0) {
            targetChassisSpeeds.vxMetersPerSecond = 0;
        }
        if (targetChassisSpeeds.vyMetersPerSecond != 0) {
            targetChassisSpeeds.vyMetersPerSecond = 0;
        }
        if (targetChassisSpeeds.omegaRadiansPerSecond != 0) {
            targetChassisSpeeds.omegaRadiansPerSecond = 0;
        }

        odometry.updateWithTime(
                GlobalSubsystem.getInstance().elapsedTime.seconds(),
                imu.getRotation2d(), new MecanumDriveWheelSpeeds(
                        frontLeft.getVelocity() * DriveConstants.DISTANCE_TO_TICKS,
                        frontRight.getVelocity() * DriveConstants.DISTANCE_TO_TICKS,
                        rearLeft.getVelocity() * DriveConstants.DISTANCE_TO_TICKS,
                        rearRight.getVelocity() * DriveConstants.DISTANCE_TO_TICKS
                )
        );

        telemetry.addData("Estimated Pose", odometry.getPoseMeters());
    }

    public void resetGyro() {
        imu.reset();
    }
}
