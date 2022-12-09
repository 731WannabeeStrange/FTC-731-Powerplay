package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {
    // Config parameters
    public static int liftLow = 330;
    public static int liftMid = 515;
    public static int liftHigh = 710;
    public static double grabPos = 0.5;
    public static double releasePos = 0.9;
    public static double waitTime = 1.5;
    public static int liftCollectPos = 150;
    public static int liftGrabPos = 0;
    public static int minHeightForArmRotation = 200;
    public static double yawArm1Default = 0;
    public static double yawArm2Default = 1;
    public static double liftPower = 0.3;

    public final Telemetry telemetry;

    public final DcMotorEx lift1;
    public final DcMotorEx lift2;
    public final ServoImplEx yawArm1;
    public final ServoImplEx yawArm2;
    public final ServoImplEx grabber;

    public Lift(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
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

    public void extendHigh() {
        lift1.setTargetPosition(liftHigh);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(liftHigh);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
    }

    public void extendMid() {
        lift1.setTargetPosition(liftMid);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(liftMid);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
    }

    public void extendLow() {
        lift1.setTargetPosition(liftLow);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(liftLow);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
    }

    public void retract() {
        setYawArmAngle(0);
        grabber.setPosition(grabPos);
        lift1.setTargetPosition(liftCollectPos);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(liftCollectPos);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
    }

    public void grab() {
        lift1.setTargetPosition(liftGrabPos);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(liftGrabPos);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
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
                angle = 90;
            } else {
                angle = 180;
            }
        }
        double pos = (-angle + 90) / 270;
        yawArm1.setPosition(pos);
        yawArm2.setPosition(1 - pos);
    }

    public int getSlidePosition() {
        return lift1.getCurrentPosition();
    }

    public int getTargetPosition() {
        return lift2.getTargetPosition();
    }
}
