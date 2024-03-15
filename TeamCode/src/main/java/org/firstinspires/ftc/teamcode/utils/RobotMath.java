package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public final class RobotMath {
    public static Translation2d getTranslationOffPoint(Translation2d startingPoint, double distance, double angle) {
        Rotation2d rotation = new Rotation2d(angle);

        Translation2d extension = getTranslationFromRotation(rotation).times(distance);
        extension = new Translation2d(extension.getY(), extension.getX());

        return startingPoint.plus(extension);
    }

    public static double getAngleToPoint(Translation2d a, Translation2d b) {
        Translation2d transformedPoint = b.minus(a);
        return Math.atan2(transformedPoint.getX(), transformedPoint.getY());
    }

    public static Translation2d getTranslationFromRotation(Rotation2d rotation) {
        return new Translation2d(rotation.getCos(), rotation.getSin());
    }
}
