package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServo;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServoPair;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

@Config
public class Lift {
    // Config parameters
    public static int liftLow = 330;
    public static int liftMid = 515;
    public static int liftHigh = 710;
    public static double grabPos = 0.5;
    public static double releasePos = 0.9;
    public static double waitTime = 1.5;
    public static int hoverPos = 150;
    public static int collectPos = 0;
    public static int minHeightForArmRotation = 200;
    public static double yawArmDefaultPos = 0.7;
    public static double liftPower = 0.3;

    public final Telemetry telemetry;

    public final DcMotorEx lift1;
    public final DcMotorEx lift2;
    public ProfiledServoPair yawArm;
    public ProfiledServo grabber;

    public Lift(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        yawArm = new ProfiledServoPair(hardwareMap, "yaw1", "yaw2", new MotionConstraint(1, 1, 1), yawArmDefaultPos);
        grabber = new ProfiledServo(hardwareMap, "grab", new MotionConstraint(1, 1, 1), grabPos);

        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
        grabber.setPosition(grabPos);
        lift1.setTargetPosition(hoverPos);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(hoverPos);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
    }

    public void retract(int pos) {
        grabber.setPosition(grabPos);
        lift1.setTargetPosition(pos);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(pos);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
    }

    public void collect() {
        lift1.setTargetPosition(collectPos);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(collectPos);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
    }

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
        yawArm.setPosition(pos);
    }

    public int getSlidePosition() {
        return lift1.getCurrentPosition();
    }

    public int getTargetPosition() {
        return lift2.getTargetPosition();
    }

    public void setPower(double power) {
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setPower(power);
        lift2.setPower(power);
    }
}
