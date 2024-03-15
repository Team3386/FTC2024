package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ToTargetCommand extends CommandBase {
    private final double target;

    private final PIDFController controller;
    private final DoubleSupplier get;
    private final DoubleConsumer set;

    public ToTargetCommand(PIDFController controller, double target, DoubleSupplier get, DoubleConsumer set) {
        this.target = target;
        this.controller = controller;
        this.get = get;
        this.set = set;
    }

    @Override
    public void initialize() {
        this.controller.setSetPoint(target);
    }

    @Override
    public void execute() {
        set.accept(controller.calculate(get.getAsDouble()));
    }

    @Override
    public boolean isFinished() {
        if (controller.atSetPoint()) {
            Log.i("ToPoint", String.format("Value at target (Target: %f, Current: %f). Finished.", controller.getSetPoint(), get.getAsDouble()));
        }
        return controller.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        set.accept(0);
    }
}
