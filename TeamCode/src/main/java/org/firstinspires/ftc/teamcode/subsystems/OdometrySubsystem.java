package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import java.util.function.Supplier;

public class OdometrySubsystem extends SubsystemBase {
    private static final OdometrySubsystem INSTANCE = new OdometrySubsystem();

    Supplier<Integer> deadWheel1, deadWheel2, deadWheel3;

    private OdometrySubsystem() {
    }

    public static OdometrySubsystem getInstance() {
        return INSTANCE;
    }

    public void init(Supplier<Integer> deadWheel1, Supplier<Integer> deadWheel2, Supplier<Integer> deadWheel3) {
        this.deadWheel1 = deadWheel1;
        this.deadWheel2 = deadWheel2;
        this.deadWheel3 = deadWheel3;
    }
}
