package org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServo;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServoPair;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double P = 0.0035;
    public static double clawOpenPos = 0.65;
    public static double clawClosedPos = 0.4;
    public static double v4bRetractedPos = 0.25;
    public static double v4bCompletelyRetractedPos = 0.1;
    public static int maxExtension = 865;
    public static int errorTolerance = 10;
    public static double coneCloseValue = 5;
    public static double[] stackPositions = {
            0.65,
            0.7,
            0.75,
            0.8,
            0.9
    };

    private final DcMotorEx slide1, slide2;
    private final ProfiledServo claw;
    private final ProfiledServoPair v4b;

    private int error1, error2;

    private RevColorSensorV3 color;

    private enum SlideState {
        RETRACT,
        EXTEND,
        STOP
    }

    private SlideState slideState = SlideState.RETRACT;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        slide1 = hardwareMap.get(DcMotorEx.class, "intake1");
        slide2 = hardwareMap.get(DcMotorEx.class, "intake2");
        v4b = new ProfiledServoPair(
                hardwareMap,
                "v4b1",
                "v4b2",
                new MotionConstraint(2, 6, 6),
                v4bRetractedPos
        );
        claw = new ProfiledServo(
                hardwareMap,
                "claw",
                new MotionConstraint(1, 4, 4),
                clawClosedPos
        );
        color = hardwareMap.get(RevColorSensorV3.class, "color");

        slide1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        switch (slideState) {
            case RETRACT:
                error1 = -slide1.getCurrentPosition();
                error2 = -slide2.getCurrentPosition();
                break;
            case EXTEND:
                error1 = maxExtension - slide1.getCurrentPosition();
                error2 = maxExtension - slide2.getCurrentPosition();
                break;
            case STOP:
                error1 = 0;
                error2 = 0;
                break;
        }

        if (Math.abs(error1) < errorTolerance && slideState != SlideState.STOP) {
            slideState = SlideState.STOP;
        }

        slide1.setPower(P * error1);
        slide2.setPower(P * error2);

        v4b.periodic();
        claw.periodic();
    }

    public void grab() {
        claw.setPosition(clawClosedPos);
    }

    public void release() {
        claw.setPosition(clawOpenPos);
    }

    public boolean isClawBusy() { return claw.isBusy(); };

    public void setV4bPos(double pos) {
        v4b.setPosition(pos);
    }

    public boolean isV4BBusy() { return v4b.isBusy(); }

    public boolean isBusy() {
        return slideState != SlideState.STOP;
    }

    public void extendFully() {
        slideState = SlideState.EXTEND;
    }

    public void retractFully() {
        v4b.setPosition(v4bRetractedPos);
        slideState = SlideState.RETRACT;
    }

    public void stopSlides() {
        slideState = SlideState.STOP;
    }

    public boolean isConeDetected() {
        return color.getDistance(DistanceUnit.CM) < 1;
    }

    public boolean isConeClose() {
        return color.getDistance(DistanceUnit.CM) < coneCloseValue;
    }
}
