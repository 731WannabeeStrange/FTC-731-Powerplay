package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServo;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServoPair;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

@Config
public class Intake extends SubsystemBase {
    // Config parameters
    public static TrapezoidProfile.Constraints intakeConstraints = new TrapezoidProfile.Constraints(1000, 1000);
    public static PIDCoefficients intakeCoefficients = new PIDCoefficients(0.0035, 0, 0);
    public static ProfiledPIDController intakeController = new ProfiledPIDController(
            intakeCoefficients.kP,
            intakeCoefficients.kI,
            intakeCoefficients.kD,
            intakeConstraints
    );
    public static double clawOpenPos = 0.65;
    public static double clawClosedPos = 0.4;
    public static double v4bExtendedPos = 0.85;
    public static double v4bRetractedPos = 0.27;
    public static int intakePartialRetract = 0;
    public static double v4bCompletelyRetractedPos = 0.15;
    public static int maxExtension = 865;
    public static int errorTolerance = 10;
    public static double slowSpeed = 0.4;
    public static double coneCloseValue = 5;
    public static double[] stackPositions = {
            0.65,
            0.7,
            0.75,
            0.8,
            0.9
    };

    private final Telemetry telemetry;

    private DcMotorEx slide1;
    private DcMotorEx slide2;

    private int error1, error2;
    private int customTarget;
    private int targetPosition;

    private RevColorSensorV3 color;

    private ProfiledServo claw;
    private ProfiledServoPair v4b;

    private DigitalChannel beamBreaker;

    private double multiplier = 1;

    private enum SlideState {
        RETRACTFULL,
        RETRACTPART,
        EXTENDING,
        EXTENDING_AUTO,
        CUSTOMEXTEND,
        STOP
    }

    private SlideState slideState = SlideState.RETRACTFULL;

    public Intake(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        slide1 = hardwareMap.get(DcMotorEx.class, "intake1");
        slide2 = hardwareMap.get(DcMotorEx.class, "intake2");
        v4b = new ProfiledServoPair(
                hardwareMap,
                "v4b1",
                "v4b2",
                new MotionConstraint(2, 4, 4),
                v4bRetractedPos
        );
        claw = new ProfiledServo(
                hardwareMap,
                "claw",
                new MotionConstraint(1, 4, 4),
                clawClosedPos
        );
        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        color = hardwareMap.get(RevColorSensorV3.class, "color");

        slide1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        beamBreaker.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void periodic() {
        v4b.periodic();
        claw.periodic();

        switch (slideState) {
            case RETRACTFULL:
                targetPosition = 0;
                break;
            case RETRACTPART:
                targetPosition = intakePartialRetract;
                break;
            case EXTENDING:
                targetPosition = maxExtension;
                break;
            case EXTENDING_AUTO:
                targetPosition = maxExtension;

                if (isConeDetected()) {
                    slideState = SlideState.STOP;
                }
                break;

            case CUSTOMEXTEND:
                targetPosition = customTarget;
                break;
            case STOP:
                targetPosition = getSlidePosition();
                break;
        }

        slide1.setPower(intakeController.calculate(slide1.getCurrentPosition(), targetPosition));
        slide2.setPower(intakeController.calculate(slide2.getCurrentPosition(), targetPosition));

        if (intakeController.atGoal() && slideState != SlideState.STOP) {
            slideState = SlideState.STOP;
        }
    }

    public void grab() {
        claw.setPosition(clawClosedPos);
    }

    public void release() {
        claw.setPosition(clawOpenPos);
    }

    public boolean isClawBusy() { return claw.isBusy(); };

    public void extendTicks(int ticks, double power, int cycle) {
        customTarget = ticks;
        slideState = SlideState.CUSTOMEXTEND;
        v4b.setPosition(stackPositions[cycle]);
    }

    public void setV4bPos(double pos) {
        v4b.setPosition(pos);
    }

    public boolean isV4BBusy() { return v4b.isBusy(); }

    public int getSlidePosition() {
        return slide1.getCurrentPosition();
    }

    public boolean isBusy() {
        return slideState != SlideState.STOP;
    }

    public void extendFully() {
        slideState = SlideState.EXTENDING;
    }

    public void extendAuto() { slideState = SlideState.EXTENDING_AUTO; }

    public void retractFully() {
        v4b.setPosition(v4bRetractedPos);
        slideState = SlideState.RETRACTFULL;
    }
    /*
    public void retractFully(double v4bpos) {
        v4b.setPosition(v4bpos);
        claw.setPosition(clawClosedPos);
        slideState = SlideState.RETRACTFULL;
    }
     */

    public void retractPart(double v4bpos) {
        v4b.setPosition(v4bpos);
        //claw.setPosition(clawClosedPos);
        slideState = SlideState.RETRACTPART;
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

    public double[] getMotorPowers() {
        return new double[]{slide1.getPower(), slide2.getPower()};
    }

    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }
}
