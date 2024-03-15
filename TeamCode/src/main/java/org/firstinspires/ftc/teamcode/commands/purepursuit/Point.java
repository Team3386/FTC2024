package org.firstinspires.ftc.teamcode.commands.purepursuit;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Constants.AutonomousConstants;

public class Point {
    private final boolean translationOnly;
    private final Pose2d pose;
    private final double motionBudget;
    private final Command command;
    private final boolean waitForCommand;

    public Point(Pose2d pose) {
        this(pose, false, null, false);
    }

    public Point(Translation2d position) {
        this(new Pose2d(position, new Rotation2d()), true, null, false);
    }

    public Point(Pose2d pose, Command command, boolean waitForCommand) {
        this(pose, false, command, waitForCommand);
    }

    public Point(Translation2d position, Command command, boolean waitForCommand) {
        this(new Pose2d(position, new Rotation2d()), true, command, waitForCommand);
    }

    public Point(Pose2d pose, boolean translationOnly, Command command, boolean waitForCommand) {
        this(pose, AutonomousConstants.MOTION_BUDGET, translationOnly, command, waitForCommand);
    }

    public Point(Pose2d pose, double motionBudget) {
        this(pose, motionBudget, false, null, false);
    }

    public Point(Pose2d pose, double motionBudget, Command command, boolean waitForCommand) {
        this(pose, motionBudget, false, command, waitForCommand);
    }

    public Point(Translation2d position, double motionBudget, Command command, boolean waitForCommand) {
        this(new Pose2d(position, new Rotation2d()), motionBudget, true, command, waitForCommand);
    }

    public Point(Pose2d pose, double motionBudget, boolean translationOnly, Command command, boolean waitForCommand) {
        this.pose = pose;
        this.motionBudget = motionBudget;
        this.translationOnly = translationOnly;
        this.command = command;
        this.waitForCommand = waitForCommand;
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

    public Command command() {
        return command;
    }

    public boolean waitForCommand() {
        return waitForCommand;
    }
}

