package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

// Units in centimeters

public final class Constants {
    public static final double CENTIMETER_PER_INCH = 2.54;

    public static class RobotConstants {
        public static final String IMU_NAME = "";
        public static final double ANALOG_NOMINAL_VOLTAGE = 3.3;
    }

    public static class ArmConstants {
        public static final String ROTATION_MOTOR_NAME = "armRotation";
        public static final String EXTEND_MOTOR_NAME = "armExtend";
        public static final String WRIST_MOTOR_NAME = "armWrist";

        public static final String ROTATION_ENCODER_NAME = "armRotationEncoder";

        public static final String TOP_LIMIT_SWITCH_NAME = "topLimit";
        public static final String BOTTOM_LIMIT_SWITCH_NAME = "bottomLimit";

        public static final double ROTATION_HIGH_DEGREE = 2540;
        public static final double ROTATION_LOW_DEGREE = 2395;
        public static final double ROTATION_PASS_DEGREE = 2418;
        public static final double ROTATION_MIDDLE_DEGREE = 2455;

        public static final double WRIST_HIGH_DEGREE = -5;
        public static final double WRIST_LOW_DEGREE = -140;
        public static final double WRIST_PARALLEL_TO_GROUND = -80;
    }

    public static class DriveConstants {
        public static final double MAX_MOVEMENT_PER_SECOND = 300;
        public static final double MAX_ROTATION_PER_SECOND = 300;

        public static final String FRONT_LEFT_MOTOR_NAME = "frontLeft";
        public static final String FRONT_RIGHT_MOTOR_NAME = "frontRight";
        public static final String REAR_LEFT_MOTOR_NAME = "rearLeft";
        public static final String REAR_RIGHT_MOTOR_NAME = "rearRight";

        public static final double GEAR_RATIO = 4 * 3;
        public static final double ENCODER_COUNT_PER_REVOLUTION = 24;

        public static final double WHEEL_DIAMETER = 3d * CENTIMETER_PER_INCH;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double DISTANCE_TO_ROTATIONS = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        public static final double DISTANCE_TO_TICKS = DISTANCE_TO_ROTATIONS / ENCODER_COUNT_PER_REVOLUTION;
        public static final double DISTANCE_TO_TICKS_INVERSE = 1d / DISTANCE_TO_TICKS;
        public static final double TICKS_TO_DISTANCE = ENCODER_COUNT_PER_REVOLUTION * GEAR_RATIO;
        public static final double TICKS_TO_DISTANCE_INVERSE = 1d / TICKS_TO_DISTANCE;


        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = 39.25;
        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = 35.25;

        public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front left
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front right
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Rear left
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // Rear right
    }

    public static class HandConstants {
        public static final String LEFT_SERVO_NAME = "handLeft";
        public static final String RIGHT_SERVO_NAME = "handRight";
        public static final double LEFT_OPEN_POSITION = 0.45;
        public static final double RIGHT_OPEN_POSITION = 0.5;
    }

    public static class PlaneConstants {
        public static final String PLANE_SERVO_NAME = "planeServo";
    }

    public static class VisionConstants {
        public static final String YOLO_MODEL_FILENAME = "mobilenet_optimized_metadata.tflite";

        public static final Map<Integer, Translation2d> APRILTAG_POSITIONS;

        static {
            Map<Integer, Translation2d> positionMap = new HashMap<>();
            // Blue alliance wall
            // Big
            positionMap.put(10, new Translation2d(15, 16));
            // Small
            // positionMap.put(9, new Translation2d(15, 16));

            // Red alliance wall
            // Big
            positionMap.put(7, new Translation2d(15, 16));
            // Small
            // positionMap.put(8, new Translation2d(15, 16));
            APRILTAG_POSITIONS = Collections.unmodifiableMap(positionMap);
        }
    }
}
