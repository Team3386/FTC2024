package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public final class RobotMath {
    public final static double EPSILON = 1e-6;

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static double getDistanceAlongPoseLine(Pose2d pose, Translation2d other) {
        Translation2d relativeVector = new Translation2d(other.getX() - pose.getX(), other.getY() - pose.getY());
        return pose.getRotation().getCos() * relativeVector.getX() + pose.getRotation().getSin() * relativeVector.getY();
    }

    public static Pose2d getPointAlongPoseLine(Pose2d pose, double distance) {
        double deltaX = pose.getRotation().getCos() * distance;
        double deltaY = pose.getRotation().getSin() * distance;
        double updatedX = pose.getX() + deltaX;
        double updatedY = pose.getY() + deltaY;

        return new Pose2d(new Translation2d(updatedX, updatedY), new Rotation2d(deltaX, deltaY));
    }

    public static Translation2d getTranslationOffPoint(Translation2d startingPoint, double distance, double angle) {
        Rotation2d rotation = new Rotation2d(angle);

        Translation2d extension = getTranslationFromRotation(rotation).times(distance);
        extension = new Translation2d(extension.getY(), extension.getX());

        return startingPoint.plus(extension);
    }

    public static Translation2d getPointAlongPoseClosestToPoint(Pose2d pose, Translation2d other) {
        // Found by taking the derivative of the distance between any point along the
        // projected pose, denoted by a distance P along the line, and the "other" point:
        //     d/dP(D(P)=sqrt((sin(a)*P-y)^2+(cos(a)*P-x)^2))
        // Then finding the zero of the resultant function, denoting the position along
        // the projected line which is closest to the target:
        //     solve (-x cos(a) - y sin(a) + P)/sqrt(-2 P x cos(a) - 2 P y sin(a) + P^2 + x^2 + y^2) for P
        // ...and finally using that distance formula to construct the final point:
        //     (cos(a)*P, sin(a)*P), for P=xcos(a)+ysin(a)
        double distanceAlongPoseLine = getDistanceAlongPoseLine(pose, other);

        return new Translation2d(
                pose.getX() + pose.getRotation().getCos() * distanceAlongPoseLine,
                pose.getY() + pose.getRotation().getSin() * distanceAlongPoseLine);
    }

    public static double getDistanceToLineFromPoint(Pose2d pose, Translation2d currentPoint) {
        return getPointAlongPoseClosestToPoint(pose, currentPoint).getDistance(currentPoint);
    }

    public static Pose2d getRabbitPose(Pose2d pose, Translation2d other, double lookaheadDistance) {
        Translation2d closestPoint = getPointAlongPoseClosestToPoint(pose, other);
        Translation2d rabbitLocation = closestPoint.plus(new Translation2d(lookaheadDistance * pose.getRotation().getCos(), lookaheadDistance * pose.getRotation().getSin()));
        return new Pose2d(rabbitLocation, pose.getRotation());
    }

    public static double getAngleToPoint(Translation2d a, Translation2d b) {
        Translation2d transformedPoint = b.minus(a);
        return Math.atan2(transformedPoint.getX(), transformedPoint.getY());
    }

    public static double getAngleToPoint2(Translation2d a, Translation2d b) {
        Translation2d transformedPoint = b.minus(a);
        return Math.atan2(transformedPoint.getY(), transformedPoint.getX());
    }

    public static Vector2d getVectorFromRotation(Rotation2d rotation) {
        return new Vector2d(rotation.getCos(), rotation.getSin());
    }

    public static Translation2d getTranslationFromRotation(Rotation2d rotation) {
        return new Translation2d(rotation.getCos(), rotation.getSin());
    }

    public static Vector2d getVectorToRabbit(Pose2d a, Pose2d b, double lookaheadDistance) {
        Pose2d rabbitPose = getRabbitPose(a, b.getTranslation(), lookaheadDistance);
        return translationToVector(rabbitPose.getTranslation().minus(b.getTranslation()));
    }

    public static Vector2d translationToVector(Translation2d translation) {
        return new Vector2d(translation.getX(), translation.getY());
    }
}
