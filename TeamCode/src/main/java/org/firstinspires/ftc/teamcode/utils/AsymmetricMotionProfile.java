package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.profile.MotionState;

public class AsymmetricMotionProfile {

    public final double initialPosition;
    public final double finalPosition;
    public final MotionConstraint constraints;
    protected double accelTime;
    protected double coastTime;
    protected double decelTime;
    protected double profileDuration;
    protected double distance;

    public AsymmetricMotionProfile(double initialPosition, double finalPosition, MotionConstraint constraints) {
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.constraints = constraints;
        computeProfileTimes();
    }

    protected void computeProfileTimes() {
        distance = finalPosition - initialPosition;

        this.accelTime = Math.abs(constraints.maxVelo) / Math.abs(constraints.maxAccel);
        this.decelTime = Math.abs(constraints.maxVelo) / Math.abs(constraints.maxDecel);

        double averageDt = (this.accelTime + this.decelTime) / 2;
        this.coastTime = Math.abs(distance) / Math.abs(constraints.maxVelo) - averageDt;

        if (this.coastTime < 0) {
            this.coastTime = 0;
            if (Math.abs(this.constraints.maxAccel) > Math.abs(constraints.maxDecel)) {
                constraints.maxAccel = Math.abs(constraints.maxDecel);
            }
            else {
                constraints.maxDecel = Math.abs(constraints.maxAccel);
            }

            this.accelTime = Math.sqrt(Math.abs(distance)/Math.abs(constraints.maxAccel));
            this.decelTime = Math.sqrt(Math.abs(distance)/Math.abs(constraints.maxDecel));
        }

        this.profileDuration = this.accelTime + this.coastTime + this.decelTime;
    }

    public MotionState getState(double seconds) {
        double acceleration;
        double velocity;
        double position;

        if (seconds <= this.accelTime) {
            acceleration = Math.abs(constraints.maxAccel);
            velocity = seconds * acceleration;
            position = 0.5 * acceleration * Math.pow(seconds, 2);
        } else if (seconds <= this.accelTime + this.coastTime) {
            acceleration = 0;
            velocity = Math.abs(getState(this.accelTime).getV());
            position = Math.abs(getState(this.accelTime).getX()) + constraints.maxVelo * (seconds - this.accelTime);
        } else if (seconds <= this.accelTime + this.coastTime + this.decelTime) {
            acceleration = Math.abs(constraints.maxDecel);
            double coastVelocity = Math.abs(getState(this.accelTime).getV());
            velocity = coastVelocity - (seconds - this.accelTime - this.coastTime) * acceleration;
            double endOfCoastTime = this.accelTime + this.coastTime;
            double endOfCoastPosition = Math.abs(getState(endOfCoastTime).getX());
            position = endOfCoastPosition + coastVelocity * (seconds - endOfCoastTime) - 0.5 * acceleration * Math.pow(seconds - endOfCoastTime, 2);
            acceleration *= -1;
        } else {
            acceleration = 0;
            velocity = 0;
            position = distance;
        }

        return new MotionState(this.initialPosition + position * Math.signum(distance),
                velocity * Math.signum(distance), acceleration * Math.signum(distance));
    }

    public double getProfileDuration() {
        return profileDuration;
    }
}