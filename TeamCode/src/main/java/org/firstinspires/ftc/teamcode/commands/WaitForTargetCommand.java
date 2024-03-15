package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class WaitForTargetCommand extends CommandBase {
    private final double target;
    private final double tolerance;
    private final DoubleSupplier get;

    public WaitForTargetCommand(double target, double tolerance, DoubleSupplier get) {
        this.target = target;
        this.tolerance = tolerance;
        this.get = get;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(get.getAsDouble() - target) < tolerance;
    }
}
