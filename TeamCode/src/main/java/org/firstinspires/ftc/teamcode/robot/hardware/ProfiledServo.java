package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

public class ProfiledServo {
    public Servo servo;
    protected double endPosition;
    protected double previousEndPosition;
    public double currentPosition;
    protected String name;
    public double initialPosition;

    public AsymmetricMotionProfile profile;
    public MotionConstraint constraints;
    public ElapsedTime timer = new ElapsedTime();

    public ProfiledServo(HardwareMap hwMap, String name, MotionConstraint constraints, double initialPosition) {
        servo = hwMap.get(Servo.class, name);
        this.name = name + " ";
        this.endPosition = initialPosition;
        this.currentPosition = initialPosition;
        this.previousEndPosition = initialPosition + 100;
        this.constraints = constraints;
        this.profile = new AsymmetricMotionProfile(initialPosition, initialPosition, constraints);
        setPosition(initialPosition);
    }

    protected void regenerate_profile() {
        profile = new AsymmetricMotionProfile(this.currentPosition, this.endPosition, constraints);
        initialPosition = this.currentPosition;
        timer.reset();
    }

    public void periodic() {
        if (endPosition != previousEndPosition) {
            regenerate_profile();
        }
        previousEndPosition = endPosition;
        int multiplier = initialPosition > endPosition ? -1 : 1;
        currentPosition = multiplier * profile.getState(timer.seconds()).getX() + initialPosition;
        servo.setPosition(currentPosition);
        Dashboard.packet.put(name + "position", currentPosition);
    }

    public boolean isBusy() {
        return timer.seconds() < profile.getProfileDuration();
    }

    public void setPosition(double endPosition) {
        this.endPosition = endPosition;
    }
}