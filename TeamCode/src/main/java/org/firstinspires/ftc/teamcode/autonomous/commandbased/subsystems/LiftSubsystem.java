package org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServoPair;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static int liftLow = 900;
    public static int liftMid = 1750;
    public static int liftHigh = 2450;
    public static double grabPos = 0.45;
    public static double releasePos = 0.9;
    public static double waitTime = 1.5;
    public static int hoverPos = 800;
    public static int collectPos = 200;
    public static int minHeightForArmRotation = 200;
    public static double P = 0.005;
    public static int errorTolerance = 10;
    public static double grabTime = 0.75;
    public static double yawArmAngle = -10;

    private int targetPosition = 0;
    private int error1 = 0;
    private int error2 = 0;
    private double currentYawArmAngle = 0;

    private final DcMotorEx lift1;
    private final DcMotorEx lift2;
    private final ProfiledServoPair yawArm;
    private final Servo grabber;

    private Lift.LiftState liftState = Lift.LiftState.RETRACT;

    private enum GrabberState {
        HOLD,
        RELEASE,
        DEPOSITING
    }

    private GrabberState grabberState = GrabberState.HOLD;

    public LiftSubsystem(HardwareMap hardwareMap) {
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        yawArm = new ProfiledServoPair(
                hardwareMap,
                "yaw1",
                "yaw2",
                new MotionConstraint(3, 4, 3),
                (0.0037037 * -45) + 0.33333
        );

        grabber = hardwareMap.get(Servo.class, "grab");
        grabber.setPosition(grabPos);

        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setLiftState(Lift.LiftState state) {
        liftState = state;
    }

    public void closeGrabber() { grabberState = GrabberState.HOLD; }

    public void openGrabber() { grabberState = GrabberState.RELEASE; }

    public void setYawArmAngle(double angle) {
        currentYawArmAngle = angle;

        while (angle < -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        if (angle < -90) {
            if (angle > -135) {
                angle = -90;
            } else {
                angle = -180;
            }
        }
        double pos = (0.0037037 * angle) + 0.33333;
        yawArm.setPosition(pos);
    }

    public boolean isBusy() { return Math.abs(error1) >= errorTolerance || Math.abs(error2) >= errorTolerance; }

    public boolean isYawArmBusy() { return yawArm.isBusy(); }

    public boolean canControlArm() {
        return lift1.getCurrentPosition() > minHeightForArmRotation;
    }

    public void periodic() {
        switch (liftState) {
            case HIGH:
                targetPosition = liftHigh;
                break;
            case MID:
                targetPosition = liftMid;
                break;
            case LOW:
                targetPosition = liftLow;
                break;
            case RETRACT:
                grabber.setPosition(grabPos);
                setYawArmAngle(yawArmAngle);
                if (!isYawArmBusy()) {
                    targetPosition = hoverPos;
                }
                break;
            case COLLECT:
                setYawArmAngle(yawArmAngle);
                targetPosition = collectPos;
                break;
            case ZERO:
                setYawArmAngle(-45);
                targetPosition = 0;
                break;
        }
        error1 = targetPosition - lift1.getCurrentPosition();
        error2 = targetPosition - lift2.getCurrentPosition();

        lift1.setPower(P * error1);
        lift2.setPower(P * error2);

        switch (grabberState) {
            case HOLD:
                grabber.setPosition(grabPos);
                break;
            case RELEASE:
                grabber.setPosition(releasePos);
                break;
        }

        yawArm.periodic();
    }
}
