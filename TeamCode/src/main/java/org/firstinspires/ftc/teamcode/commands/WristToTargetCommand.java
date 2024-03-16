package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Constants.AutonomousConstants;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class WristToTargetCommand extends ToTargetCommand {
    public WristToTargetCommand(PIDFController controller, double target, DoubleSupplier get, DoubleConsumer set) {
        super(controller, target, AutonomousConstants.WRIST_LIMIT, get, set);
    }
}
