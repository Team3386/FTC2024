package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AlignWithApriltag;

@Autonomous
public class AprilTagTest extends Robot {
    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(new AlignWithApriltag(9));
    }
}
