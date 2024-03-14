package org.firstinspires.ftc.teamcode.commands.purepursuit;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Constants.AutonomousConstants;

public class Point {
    private final boolean translationOnly;
    private final Pose2d pose;
    private final double motionBudget;

    public Point(Pose2d pose) {
        this(pose, false);
    }

    public Point(Translation2d position) {
        this(new Pose2d(position, new Rotation2d()), true);
    }

    public Point(Pose2d pose, boolean translationOnly) {
        this(pose, AutonomousConstants.MOTION_BUDGET, translationOnly);
    }

    public Point(Pose2d pose, double motionBudget) {
        this(pose, motionBudget, false);
    }

    public Point(Translation2d position, double motionBudget) {
        this(new Pose2d(position, new Rotation2d()), motionBudget, true);
    }

    public Point(Pose2d pose, double motionBudget, boolean translationOnly) {
        this.pose = pose;
        this.motionBudget = motionBudget;
        this.translationOnly = translationOnly;
    }

    public Pose2d pose() {
        return pose;
    }

    public double motionBudget() {
        return motionBudget;
    }

    public boolean translationOnly() {
        return translationOnly;
    }
}

