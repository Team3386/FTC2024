package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

// Units in centimeters

@Config
public class Constants {
    public static double CENTIMETER_PER_INCH = 2.54;
    public static double CENTIMETER_PER_INCH_INVERSE = 1 / CENTIMETER_PER_INCH;

    @Config
    public static class AutonomousConstants {
        public static double[] POSITION_PID = {5, 0, 0};
        public static double[] ROTATION_PID = {150, 0, 0};
        public static double MOTION_BUDGET = 150;

        public static double[] ARM_ROTATION_PID = {1, 0, 0};
        public static double[] ARM_WRIST_PID = {0.007, 0, 0};
        public static double[] ARM_EXTEND_PID = {0.2, 0, 0};

        public static double ARM_ROTATION_PASS = 25;
        public static double ARM_ROTATION_BACKBOARD = 40;
        public static double ARM_ROTATION_PICKUP = 0;

        public static double ARM_EXTEND_TOP = 210;
        public static double ARM_EXTEND_BOTTOM = 132;
        public static double ARM_EXTEND_PICKUP = 160;

        public static double ARM_WRIST_UP = -600;
        public static double ARM_WRIST_DOWN = 0;
        public static double ARM_WRIST_PICKUP = -480;

        public static double WRIST_LIMIT = 0.5;

        public static long WAIT_BEFORE_UP = 300;
        public static long WAIT_BEFORE_LOCK = 2000;
    }

    @Config
    public static class RobotConstants {
        public static String IMU_NAME = "";
        public static double ANALOG_NOMINAL_VOLTAGE = 3.3;
    }

    @Config
    public static class ArmConstants {
        public static String ROTATION_MOTOR_NAME = "armRotation";
        public static String EXTEND_MOTOR_NAME = "armExtend";
        public static String WRIST_MOTOR_NAME = "armWrist";

        public static String ROTATION_ENCODER_NAME = "armRotationEncoder";
        public static String EXTEND_ENCODER_NAME = "armExtendEncoder";

        public static String TOP_LIMIT_SWITCH_NAME = "topLimit";
        public static String BOTTOM_LIMIT_SWITCH_NAME = "bottomLimit";

//        public static double ROTATION_HIGH_DEGREE = 2540;
//        public static double ROTATION_LOW_DEGREE = 2395;
//        public static double ROTATION_PASS_DEGREE = 2418;
//        public static double ROTATION_MIDDLE_DEGREE = 2455;

        public static double ROTATION_HIGH_DEGREE = 130;
        public static double ROTATION_LOW_DEGREE = 0;
        public static double ROTATION_PASS_DEGREE = 28;

        public static double WRIST_HIGH_DEGREE = 49;
        public static double WRIST_LOW_DEGREE = 2;
        public static double WRIST_PARALLEL_TO_GROUND = -80;

        public static double EXTEND_HIGH_DEGREE = 9999999;
        public static double EXTEND_LOW_DEGREE = -9999999;
    }

    @Config
    public static class DriveConstants {
        public static double MAX_MOVEMENT_PER_SECOND = 300;
        public static double MAX_ROTATION_PER_SECOND = 300;

        public static double[] MOTOR_PID = {1, 0, 0.0075};

        public static String FRONT_LEFT_MOTOR_NAME = "frontLeft";
        public static String FRONT_RIGHT_MOTOR_NAME = "frontRight";
        public static String REAR_LEFT_MOTOR_NAME = "rearLeft";
        public static String REAR_RIGHT_MOTOR_NAME = "rearRight";

        public static double GEAR_RATIO = 4 * 3;
        public static double ENCODER_COUNT_PER_REVOLUTION = 24;

        public static double WHEEL_DIAMETER = 3d * CENTIMETER_PER_INCH;
        public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static double DISTANCE_TO_ROTATIONS = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        public static double DISTANCE_TO_TICKS = DISTANCE_TO_ROTATIONS / ENCODER_COUNT_PER_REVOLUTION;
        public static double DISTANCE_TO_TICKS_INVERSE = 1d / DISTANCE_TO_TICKS;
        public static double TICKS_TO_DISTANCE = ENCODER_COUNT_PER_REVOLUTION * GEAR_RATIO;
        public static double TICKS_TO_DISTANCE_INVERSE = 1d / TICKS_TO_DISTANCE;


        // Distance between centers of right and left wheels on robot
        public static double TRACK_WIDTH = 39.25;
        // Distance between front and back wheels on robot
        public static double WHEEL_BASE = 35.25;

        public static MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front left
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front right
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Rear left
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // Rear right
    }

    @Config
    public static class HandConstants {
        public static String LEFT_SERVO_NAME = "handLeft";
        public static String RIGHT_SERVO_NAME = "handRight";
        public static double LEFT_OPEN_POSITION = 0;
        public static double RIGHT_OPEN_POSITION = 0;
    }

    @Config
    public static class PlaneConstants {
        public static String PLANE_SERVO_NAME = "planeServo";
    }

    @Config
    public static class OdometryConstants {
        public static String LEFT_ENCODER_NAME = "armRotation";
        public static String RIGHT_ENCODER_NAME = "armExtend";
        public static String PERP_ENCODER_NAME = "perpEncoder";

        public static double TRACK_WIDTH = 19.275;
        public static double CENTER_WHEEL_OFFSET = -13.70784;
//        public static double CENTER_WHEEL_OFFSET = -12.15;

        public static double COUNT_PER_REVOLUTION = 2000;

        public static double WHEEL_DIAMETER = 48 * 0.1;
        public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static double TICKS_TO_DISTANCE = WHEEL_CIRCUMFERENCE / COUNT_PER_REVOLUTION;

        public static double[] DEAD_WHEEL_POSITIONS = {
                -TRACK_WIDTH / 2, // Left X
                TRACK_WIDTH / 2, // Right X
                CENTER_WHEEL_OFFSET, // Center Y
        };
    }

    @Config
    public static class VisionConstants {
        public static String YOLO_MODEL_FILENAME = "mobilenet_optimized_metadata.tflite";

        public static Translation2d CAMERA_POSITION = new Translation2d(2, -5.5);

        public static Map<Integer, Translation2d> APRILTAG_POSITIONS;

        static {
            Map<Integer, Translation2d> positionMap = new HashMap<>();

            // Backboard blue center
//            positionMap.put(2, new Translation2d(-91.44, 152));

            // Red big
            positionMap.put(7, new Translation2d(101, -179.5));
            // Red small
            positionMap.put(8, new Translation2d(90.5, -179.5));


            // Blue big
            positionMap.put(10, new Translation2d(-101, -179.5));
            // Blue small
            positionMap.put(9, new Translation2d(-90.5, -179.5));


            APRILTAG_POSITIONS = Collections.unmodifiableMap(positionMap);
        }
    }
}
