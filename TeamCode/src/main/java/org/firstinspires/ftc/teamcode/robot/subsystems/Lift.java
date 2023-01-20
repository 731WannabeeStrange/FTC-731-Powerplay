package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServo;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServoPair;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

@Config
public class Lift {
    // Config parameters
    public static int liftLow = 800;
    public static int liftMid = 1300;
    public static int liftHigh = 1800;
    public static double grabPos = 0.5;
    public static double releasePos = 0.9;
    public static double waitTime = 1.5;
    public static int hoverPos = 884;
    public static int collectPos = 0;
    public static int minHeightForArmRotation = 200;
    public static double yawArmDefault = 0.3;

    public static double P = 0.006;
    public static int errorTolerance = 25;
    public static double grabTime = 0.75;

    public final Telemetry telemetry;

    public final DcMotorEx lift1;
    public final DcMotorEx lift2;
    public final ProfiledServoPair yawArm;
    public final ProfiledServo grabber;

    private int targetPosition = 0;
    private int error1 = 0;
    private int error2 = 0;

    private enum LiftState {
        HIGH,
        MID,
        LOW,
        RETRACT,
        COLLECT
    }

    private LiftState liftState = LiftState.RETRACT;

    private enum GrabberState {
        HOLD,
        RELEASE,
        DEPOSITING
    }

    private GrabberState grabberState = GrabberState.HOLD;

    private final ElapsedTime grabTimer = new ElapsedTime();

    public Lift(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        yawArm = new ProfiledServoPair(
                hardwareMap,
                "yaw1",
                "yaw2",
                new MotionConstraint(1, 4, 4),
                yawArmDefault
        );

        grabber = new ProfiledServo(
                hardwareMap,
                "grab",
                new MotionConstraint(1, 4, 4),
                grabPos
        );

        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void extendHigh() { liftState = LiftState.HIGH; }

    public void extendMid() { liftState = LiftState.MID; }

    public void extendLow() { liftState = LiftState.LOW; }

    public void retract() {
        liftState = LiftState.RETRACT;
    }
    public void retract(int pos) { liftState = LiftState.RETRACT; } // ignores pos argument for now,
                                                                    // can change in the future

    public void collect() {
        liftState = LiftState.COLLECT;
        update();
    }

    public void grab() { grabberState = GrabberState.HOLD; }

    public void deposit() {
        grabberState = GrabberState.RELEASE;
        update();
    }

    public void setYawArmAngle(double angle) {
        while (angle < -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        if (angle > 90) {
            if (angle < 135) {
                angle = -180;
            } else {
                angle = 90;
            }
        }
        double pos = (-angle + 90) / 270;
        yawArm.setPosition(pos);
    }

    public int getSlidePosition() { return lift1.getCurrentPosition(); }

    public int getTargetPosition() { return targetPosition; }

    public boolean isBusy() { return Math.abs(error1) >= errorTolerance || Math.abs(error2) >= errorTolerance; }

    public void update() {
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
                setYawArmAngle(0);
                targetPosition = hoverPos;
                break;
            case COLLECT:
                targetPosition = collectPos;
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
                grabberState = GrabberState.DEPOSITING;
                grabTimer.reset();
                break;
            case DEPOSITING:
                if (grabTimer.time() > grabTime) {
                    grabberState = GrabberState.HOLD;
                }
                break;
        }

        yawArm.periodic();
        grabber.periodic();
    }
}
