package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.teamcode.utils.AbstractMotorSubsystem;

public class ToTargetCommand extends CommandBase {
    private final double target;
    private final double limit;

    private final PIDFController controller;
    private final AbstractMotorSubsystem subsystem;

    public ToTargetCommand(PIDFController controller, double target, AbstractMotorSubsystem subsystem) {
        this(controller, target, 1, subsystem);
    }

    public ToTargetCommand(PIDFController controller, double target, double limit, AbstractMotorSubsystem subsystem) {
        this.target = target;
        this.limit = limit;
        this.controller = new PIDFController(controller.getP(), controller.getI(), controller.getD(), controller.getF());
        this.subsystem = subsystem;

        this.controller.setTolerance(controller.getTolerance()[0]);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        Log.i("ToPoint", String.format("Initializing with target %f, on %s", target, subsystem.getName()));
        this.controller.setSetPoint(target);
    }

    @Override
    public void execute() {
        subsystem.setMotor(MathUtils.clamp(controller.calculate(subsystem.get()), -limit, limit));
    }

    @Override
    public boolean isFinished() {
        if (controller.atSetPoint()) {
            Log.i("ToPoint", String.format("Value at target (Target: %f, Current: %f). Finished.", controller.getSetPoint(), subsystem.get()));
        }
        return controller.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setMotor(0);
    }
}
