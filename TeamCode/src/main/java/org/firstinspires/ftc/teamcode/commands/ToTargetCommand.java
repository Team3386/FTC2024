package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.MathUtils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ToTargetCommand extends CommandBase {
    private final double target;
    private final double limit;

    private final PIDFController controller;
    private final DoubleSupplier get;
    private final DoubleConsumer set;

    public ToTargetCommand(PIDFController controller, double target, DoubleSupplier get, DoubleConsumer set) {
        this(controller, target, 1, get, set);
    }

    public ToTargetCommand(PIDFController controller, double target, double limit, DoubleSupplier get, DoubleConsumer set) {
        this.target = target;
        this.limit = limit;
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
        set.accept(MathUtils.clamp(controller.calculate(get.getAsDouble()), -limit, limit));
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
