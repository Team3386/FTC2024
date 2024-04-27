package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Constants.AutonomousConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmWristSubsystem;

public class WristToTargetCommand extends ToTargetCommand {
    public WristToTargetCommand(PIDFController controller, double target) {
        super(controller, target, AutonomousConstants.WRIST_LIMIT, ArmWristSubsystem.getInstance());
    }
}
