package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ToTargetCommand extends CommandBase {
    private final PIDFController controller;
    private final DoubleSupplier get;
    private final DoubleConsumer set;

    public ToTargetCommand(PIDFController controller, double target, DoubleSupplier get, DoubleConsumer set, Subsystem subsystem) {
        this.get = get;
        this.set = set;
        this.controller = controller;
        this.controller.setSetPoint(target);
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        set.accept(controller.calculate(get.getAsDouble()));
    }

    @Override
    public boolean isFinished() {
        return controller.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        set.accept(0);
    }
}
