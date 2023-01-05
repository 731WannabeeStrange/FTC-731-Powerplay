package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServo;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServoPair;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

@Config
public class Intake extends Subsystem {
    // Config parameters
    public static double clawOpenPos = 0;
    public static double clawClosedPos = 1;
    public static double v4bRetractedPos = 0;
    public static double v4bExtendedPos = 1;
    public static int[] stackPositions = {0, 0, 0, 0, 0};

    public final Telemetry telemetry;

    public DcMotorEx slide1;
    public DcMotorEx slide2;

    public ProfiledServoPair v4b;
    public ProfiledServo claw;

    public DigitalChannel beamBreaker;

    public final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public int depositTicks;

    public boolean grabbing = false;
    public boolean previousGrabButton = false;

    public Intake(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        slide1 = hardwareMap.get(DcMotorEx.class, "intake1");
        slide2 = hardwareMap.get(DcMotorEx.class, "intake2");
        v4b = new ProfiledServoPair(hardwareMap, "v4b1", "v4b2", new MotionConstraint(1, 1, 1), v4bRetractedPos);
        claw = new ProfiledServo(hardwareMap, "claw", new MotionConstraint(1, 1, 1), clawOpenPos);
        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");

        slide1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        beamBreaker.setMode(DigitalChannel.Mode.INPUT);
    }

    public void extend(double intakeExtension, double intakeRetraction) {
        release();
        slide1.setPower(intakeExtension - intakeRetraction);
        slide2.setPower(intakeExtension - intakeRetraction);
        v4b.setPosition(v4bExtendedPos);
    }

    public void retractFully() {
        slide1.setTargetPosition(0);
        slide2.setTargetPosition(0);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4b.setPosition(v4bRetractedPos);
    }

    public void grab() {
        claw.setPosition(clawClosedPos);
    }

    public void release() {
        claw.setPosition(clawOpenPos);
    }

    public void extendTicks(int ticks, double power) {
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setTargetPosition(ticks);
        slide2.setTargetPosition(ticks);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(power);
        slide2.setPower(power);
    }

    public int getSlidePosition() {
        return slide1.getCurrentPosition();
    }

    public void periodic() {
        v4b.periodic();
        claw.periodic();
    }
}
