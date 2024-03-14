package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class RotationPIDFController extends PIDFController {
    private static final double PI_2 = Math.PI / 2;

    private Rotation2d targetHeading = new Rotation2d();
    private boolean continuous;

    public RotationPIDFController(double kp, double ki, double kd, double kf) {
        super(kp, ki, kd, kf);
        super.setSetPoint(0);
    }

    public void setContinuous(boolean val) {
        continuous = val;
    }

    @Override
    public double calculate(double pv, double sp) {
        targetHeading = new Rotation2d(sp);
        return calculate(pv);
    }

    public double calculate(Rotation2d pv, Rotation2d sp) {
        targetHeading = sp;
        return calculate(pv);
    }

    @Override
    public double calculate(double pv) {
        return calculate(new Rotation2d(pv));
    }

    public double calculate(Rotation2d pv) {
        // We need to calculate our own error function. Why?`
        // PID works great, but it assumes there is a linear relationship between your current state and
        // your target state. Since rotation is circular, that's not the case: if you are at 170 degrees,
        // and you want to go to -170 degrees, you could travel -340 degrees... or just +20.

        // So, we perform our own error calculation here that takes that into account (thanks to the ContiguousDouble
        // class, which is aware of such circular effects), and then feed that into a PID where
        // Goal is 0 and Current is our error.
        Rotation2d error = targetHeading.minus(pv);

        if (continuous && Math.abs(error.getRadians()) > PI_2) {
            error = new Rotation2d(error.getRadians() - Math.copySign(Math.PI, error.getRadians()));
        }

        // Now we feed it into a PID system, where the goal is to have 0 error.
        return -super.calculate(error.getRadians());
    }
}
