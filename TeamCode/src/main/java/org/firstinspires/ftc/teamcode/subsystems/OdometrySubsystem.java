package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.CENTER_WHEEL_OFFSET;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.OdometryConstants;

public class OdometrySubsystem extends SubsystemBase {
    private static final OdometrySubsystem INSTANCE = new OdometrySubsystem();
    private Pose2d calculatedPose = new Pose2d();
    private Pose2d finalPose = new Pose2d();
    private MotorEx leftEncoder, rightEncoder, centerEncoder;
    private double prevLeftEncoder, prevRightEncoder, prevCenterEncoder;
    private Twist2d twistSinceLastVision = new Twist2d();
    private Rotation2d lastVisionRotation = new Rotation2d();
    private double prevRotation;

    private OdometrySubsystem() {
    }

    public static OdometrySubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;

        leftEncoder = new MotorEx(hardwareMap, OdometryConstants.LEFT_ENCODER_NAME);
        rightEncoder = new MotorEx(hardwareMap, OdometryConstants.RIGHT_ENCODER_NAME);
        centerEncoder = new MotorEx(hardwareMap, OdometryConstants.PERP_ENCODER_NAME);

        leftEncoder.setDistancePerPulse(OdometryConstants.TICKS_TO_DISTANCE);
        rightEncoder.setDistancePerPulse(OdometryConstants.TICKS_TO_DISTANCE);
        centerEncoder.setDistancePerPulse(OdometryConstants.TICKS_TO_DISTANCE);

        resetPose(new Pose2d());
    }

    public void resetPose(Pose2d newPose) {
        leftEncoder.resetEncoder();
        rightEncoder.resetEncoder();
        centerEncoder.resetEncoder();
        prevCenterEncoder = 0;
        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevRotation = DriveSubsystem.getInstance().getPose().getHeading();
        calculatedPose = new Pose2d(newPose.getTranslation(), newPose.getRotation().unaryMinus());
        finalPose = newPose;
    }

    public void visionResetPosition(Translation2d estimatedPosition) {
        calculatedPose = new Pose2d(estimatedPosition, lastVisionRotation).exp(twistSinceLastVision);
    }

    public void resetVisionStore() {
        twistSinceLastVision = new Twist2d();
        lastVisionRotation = calculatedPose.getRotation();
    }

    public Pose2d getPose() {
        return finalPose;
    }

    @Override
    public void periodic() {
        calculatePose();

        TelemetryPacket fieldPacket = GlobalSubsystem.getInstance().fieldPacket;

        fieldPacket.fieldOverlay().setTranslation(finalPose.getY() * Constants.CENTIMETER_PER_INCH_INVERSE, -finalPose.getX() * Constants.CENTIMETER_PER_INCH_INVERSE);

        fieldPacket.fieldOverlay()
                .setStroke("red")
                .setRotation(-finalPose.getHeading())
                .strokeRect(-DriveConstants.TRACK_WIDTH / 2, -DriveConstants.WHEEL_BASE / 2, DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE);

        GlobalSubsystem.getInstance().telemetry.addData("Odometry: Estimated Pose", calculatedPose);
    }

    private void calculatePose() {
        final Pose2d drivePose = DriveSubsystem.getInstance().getPose();

        final double leftDelta = leftEncoder.getDistance() - prevLeftEncoder;
        // TODO: horrible hack fix later plz
        final double rightDelta = -(rightEncoder.getDistance() - prevRightEncoder);
        final double centerDelta = centerEncoder.getDistance() - prevCenterEncoder;
//        final Rotation2d currentRotation = prevRotation.plus(new Rotation2d((leftDelta - rightDelta) / OdometryConstants.TRACK_WIDTH));
        final double deltaRot = drivePose.getHeading() - prevRotation;

        prevLeftEncoder = leftEncoder.getDistance();
        prevRightEncoder = rightEncoder.getDistance();
        prevCenterEncoder = centerEncoder.getDistance();
        prevRotation = drivePose.getHeading();

        Telemetry telemetry = GlobalSubsystem.getInstance().telemetry;

        telemetry.addData("Odometry: Delta left", leftDelta);
        telemetry.addData("Odometry: Delta right", rightDelta);
        telemetry.addData("Odometry: Delta center", centerDelta);

        final double deltaX = centerDelta - (CENTER_WHEEL_OFFSET * -deltaRot);
        final double deltaY = -(leftDelta + rightDelta) / 2;

        Twist2d twist = new Twist2d(-deltaX, deltaY, -deltaRot);

        twistSinceLastVision = new Twist2d(twistSinceLastVision.dx + -deltaX, twistSinceLastVision.dy + deltaY, twistSinceLastVision.dtheta + -deltaRot);

        calculatedPose = calculatedPose.exp(twist);
        calculatedPose = new Pose2d(calculatedPose.getTranslation(), drivePose.getRotation().unaryMinus());
        finalPose = new Pose2d(calculatedPose.getTranslation(), drivePose.getRotation());
    }
}
