package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServo;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServoPair;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

@Config
public class Intake {
    // Config parameters
    public static double clawOpenPos = 0;
    public static double clawClosedPos = 1;
    public static double v4bRetractedPos = 0.2;
    public static int maxExtension = 780;
    public static double[] stackPositions = {
            0.55,
            0.6,
            0.65,
            0.72,
            0.72
    };

    public final Telemetry telemetry;

    public DcMotorEx slide1;
    public DcMotorEx slide2;

    public ProfiledServo claw;
    public ProfiledServoPair v4b;

    public DigitalChannel beamBreaker;

    public final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public int depositTicks;

    public boolean grabbing = false;
    public boolean previousGrabButton = false;

    public Intake(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        slide1 = hardwareMap.get(DcMotorEx.class, "intake1");
        slide2 = hardwareMap.get(DcMotorEx.class, "intake2");
        v4b = new ProfiledServoPair(
                hardwareMap,
                "v4b1",
                "v4b2",
                new MotionConstraint(1, 4, 4),
                v4bRetractedPos
        );
        claw = new ProfiledServo(
                hardwareMap,
                "claw",
                new MotionConstraint(1, 4, 4),
                clawOpenPos
        );
        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");

        slide1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        beamBreaker.setMode(DigitalChannel.Mode.INPUT);
    }

    public void update() {
        v4b.periodic();
        claw.periodic();
    }

    public void extend(double intakeExtension, double intakeRetraction) {
        release();
        slide1.setPower(intakeExtension - intakeRetraction);
        slide2.setPower(intakeExtension - intakeRetraction);
        v4b.setPosition(stackPositions[4]);
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

    public void extendTicks(int ticks, double power, int cycle) {
        slide1.setTargetPosition(ticks);
        slide2.setTargetPosition(ticks);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(power);
        slide2.setPower(power);
        v4b.setPosition(stackPositions[cycle]);
    }

    public void setV4bPos(double pos) {
        v4b.setPosition(pos);
    }

    public int getSlidePosition() {
        return slide1.getCurrentPosition();
    }

    public boolean isBusy() {
        return Math.abs(getSlidePosition() - slide1.getTargetPosition()) > 10;
    }
}
