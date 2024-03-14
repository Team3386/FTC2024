package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constants;
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

        frontLeft.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);
        frontRight.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);
        rearLeft.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);
        rearRight.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);

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

        imu = new RevIMU(globalSubsystem.hardwareMap, "navx");
        imu.invertGyro();
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, Rotation2d rotateBy) {
        targetChassisSpeeds = fieldRelative ?
                fromFieldRelativeSpeeds(
                        chassisSpeeds, odometry.getPoseMeters().getRotation().plus(rotateBy))
                : chassisSpeeds;
    }

    private ChassisSpeeds fromFieldRelativeSpeeds(ChassisSpeeds chassisSpeeds, Rotation2d robotAngle) {
        return new ChassisSpeeds(
                chassisSpeeds.vxMetersPerSecond * robotAngle.getCos() - chassisSpeeds.vyMetersPerSecond * robotAngle.getSin(),
                chassisSpeeds.vyMetersPerSecond * robotAngle.getCos() + chassisSpeeds.vxMetersPerSecond * robotAngle.getSin(),
                chassisSpeeds.omegaRadiansPerSecond
        );
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
        telemetry.addData("IMU raw orientation", imu.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES));

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

        MecanumDriveWheelSpeeds realWheelSpeeds = new MecanumDriveWheelSpeeds(
                frontLeft.getVelocity() * DriveConstants.DISTANCE_TO_TICKS,
                frontRight.getVelocity() * DriveConstants.DISTANCE_TO_TICKS,
                rearLeft.getVelocity() * DriveConstants.DISTANCE_TO_TICKS,
                rearRight.getVelocity() * DriveConstants.DISTANCE_TO_TICKS
        );

        odometry.updateWithTime(
                GlobalSubsystem.getInstance().elapsedTime.seconds(),
                imu.getRotation2d(), realWheelSpeeds
        );

        ChassisSpeeds robotState = DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(realWheelSpeeds);

        Pose2d odometryPose = odometry.getPoseMeters();

        TelemetryPacket fieldPacket = GlobalSubsystem.getInstance().fieldPacket;

        fieldPacket.fieldOverlay().setTranslation(odometryPose.getY() * Constants.CENTIMETER_PER_INCH_INVERSE, odometryPose.getX() * Constants.CENTIMETER_PER_INCH_INVERSE);

        fieldPacket.fieldOverlay()
                .setStroke("blue")
                .setRotation(-odometryPose.getHeading())
                .strokeRect(-DriveConstants.TRACK_WIDTH / 2, -DriveConstants.WHEEL_BASE / 2, DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE);

        fieldPacket.fieldOverlay().setStroke("green")
                .strokeLine(0, 0, -robotState.vyMetersPerSecond, 0);

        fieldPacket.fieldOverlay().setStroke("red")
                .strokeLine(0, 0, 0, -robotState.vxMetersPerSecond);

        fieldPacket.fieldOverlay().setStroke("white").strokeLine(0, 0, -robotState.vyMetersPerSecond, -robotState.vxMetersPerSecond);

        telemetry.addData("Estimated Pose", odometryPose);
    }

    public void resetGyro() {
        imu.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(newPose, imu.getRotation2d());
    }
}
