package org.firstinspires.ftc.teamcode.utils;

public class MotionConstraint {
    public double maxVelo;
    public double maxAccel;
    public double maxDecel;

    public MotionConstraint(double maxVelo, double maxAccel, double maxDecel) {
        this.maxVelo = maxVelo;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
    }
}
