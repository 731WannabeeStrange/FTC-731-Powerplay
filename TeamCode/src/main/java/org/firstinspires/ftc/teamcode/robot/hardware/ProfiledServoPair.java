package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

public class ProfiledServoPair {
    public ProfiledServo servo1, servo2;
    protected double endPosition;
    protected double previousEndPosition;
    protected double currentPosition;
    public double initialPosition;
    protected String name;

    public AsymmetricMotionProfile profile;
    public MotionConstraint constraints;
    public ElapsedTime timer = new ElapsedTime();

    public ProfiledServoPair(HardwareMap hwMap, String name1, String name2, MotionConstraint constraints, double initialPosition) {
        this.servo1 = new ProfiledServo(hwMap, name1, constraints, initialPosition);
        this.servo2 = new ProfiledServo(hwMap, name2, constraints,  1 - initialPosition);
        this.name = servo1.name + " " + servo2.name + " pair ";
        this.endPosition = initialPosition;
        this.currentPosition = initialPosition;
        this.previousEndPosition = initialPosition + 100; // just guarantee that they are not equal
        this.constraints = constraints;
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
        setPositionsSynced(currentPosition);
        Dashboard.packet.put(name + "position", currentPosition);
    }

    public boolean isBusy() {
        return timer.seconds() < profile.getProfileDuration();
    }

    protected void setPositionsSynced(double pos) {
        servo1.setPosition(1 - pos);
        servo2.setPosition(pos);
    }

    public void setPosition(double endPosition) {
        this.endPosition = endPosition;
    }
}