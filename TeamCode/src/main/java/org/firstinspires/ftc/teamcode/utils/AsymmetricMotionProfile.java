package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.profile.MotionState;

public class AsymmetricMotionProfile {

    public final double initialPosition;
    public final double finalPosition;
    public final MotionConstraint constraints;
    public double accelTime;
    public double coastTime;
    public double decelTime;
    protected double profileDuration = 0;
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

        this.coastTime = Math.abs(distance) / Math.abs(constraints.maxVelo) - (accelTime + decelTime) / 2;

        if (this.coastTime < 0) {
            this.coastTime = 0;
            if (Math.abs(this.constraints.maxAccel) > Math.abs(constraints.maxDecel)) {
                constraints.maxAccel = Math.abs(constraints.maxDecel);
            }
            else {
                constraints.maxDecel = Math.abs(constraints.maxAccel);
            }

            this.accelTime = Math.sqrt(Math.abs(distance) / Math.abs(constraints.maxAccel));
            this.decelTime = Math.sqrt(Math.abs(distance) / Math.abs(constraints.maxDecel));
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
            velocity = getState(this.accelTime).getV();
            position = getState(this.accelTime).getX() + constraints.maxVelo * (seconds - this.accelTime);
        } else if (seconds <= this.accelTime + this.coastTime + this.decelTime) {
            acceleration = Math.abs(constraints.maxDecel);
            double coastVelocity = Math.abs(getState(this.accelTime).getV());
            velocity = coastVelocity - (seconds - this.accelTime - this.coastTime) * acceleration;
            double endOfCoastTime = this.accelTime + this.coastTime;
            double endOfCoastPosition = getState(endOfCoastTime).getX();
            position = endOfCoastPosition + coastVelocity * (seconds - endOfCoastTime) - 0.5 * acceleration * Math.pow(seconds - endOfCoastTime, 2);
            acceleration *= -1;
        } else {
            acceleration = 0;
            velocity = 0;
            return new MotionState(Math.abs(distance),
                    velocity * Math.signum(distance), acceleration * Math.signum(distance));
        }

        return new MotionState(position, velocity, acceleration);
    }

    public double getProfileDuration() {
        return profileDuration;
    }
}