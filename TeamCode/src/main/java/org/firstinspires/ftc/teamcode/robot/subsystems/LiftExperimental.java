package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftExperimental {
    // Config parameters
    public static double P = 0.005;
    public static int liftLow = 330;
    public static int liftMid = 515;
    public static int liftHigh = 710;
    public static double grabPos = 0.5;
    public static double releasePos = 0.9;
    public static int liftHoverPos = 150;
    public static int liftCollectPos = 0;
    public static double yawArm1Default = 0.7;
    public static double yawArm2Default = 0.3;
    public static int errorTolerance = 25;

    public final Telemetry telemetry;

    public final DcMotorEx lift1;
    public final DcMotorEx lift2;
    public final ServoImplEx yawArm1;
    public final ServoImplEx yawArm2;
    public final ServoImplEx grabber;

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

    public LiftExperimental(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        yawArm1 = hardwareMap.get(ServoImplEx.class, "yaw1");
        yawArm2 = hardwareMap.get(ServoImplEx.class, "yaw2");
        grabber = hardwareMap.get(ServoImplEx.class, "grab");

        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        yawArm1.setPosition(yawArm1Default);
        yawArm2.setPosition(yawArm2Default);
        grabber.setPosition(grabPos);
    }

    public void extendHigh() { liftState = LiftState.HIGH; }

    public void extendMid() { liftState = LiftState.MID; }

    public void extendLow() { liftState = LiftState.LOW; }

    public void retract() { liftState = LiftState.RETRACT; }

    public void collect() { liftState = LiftState.COLLECT; }

    public void grab() {
        grabber.setPosition(grabPos);
    }

    public void deposit() {
        grabber.setPosition(releasePos);
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
        yawArm1.setPosition(pos);
        yawArm2.setPosition(1 - pos);
    }

    public int getSlidePosition() { return lift1.getCurrentPosition(); }

    public int getTargetPosition() { return targetPosition; }

    public boolean isBusy() { return Math.abs(error1) >= errorTolerance || Math.abs(error2) >= errorTolerance; }

    public void update() {
        switch (liftState) {
            case HIGH:
                targetPosition = liftHigh;
                error1 = targetPosition - lift1.getCurrentPosition();
                error2 = targetPosition - lift2.getCurrentPosition();
                lift1.setPower(P * error1);
                lift2.setPower(P * error2);

                break;
            case MID:
                targetPosition = liftMid;
                error1 = targetPosition - lift1.getCurrentPosition();
                error2 = targetPosition - lift2.getCurrentPosition();
                lift1.setPower(P * error1);
                lift2.setPower(P * error2);

                break;
            case LOW:
                targetPosition = liftLow;
                error1 = targetPosition - lift1.getCurrentPosition();
                error2 = targetPosition - lift2.getCurrentPosition();
                lift1.setPower(P * error1);
                lift2.setPower(P * error2);

                break;
            case RETRACT:
                grabber.setPosition(grabPos);

                targetPosition = liftHoverPos;
                error1 = targetPosition - lift1.getCurrentPosition();
                error2 = targetPosition - lift2.getCurrentPosition();
                lift1.setPower(P * error1);
                lift2.setPower(P * error2);

                break;
            case COLLECT:
                targetPosition = liftCollectPos;
                error1 = targetPosition - lift1.getCurrentPosition();
                error2 = targetPosition - lift2.getCurrentPosition();
                lift1.setPower(P * error1);
                lift2.setPower(P * error2);

                break;
        }
    }
}
