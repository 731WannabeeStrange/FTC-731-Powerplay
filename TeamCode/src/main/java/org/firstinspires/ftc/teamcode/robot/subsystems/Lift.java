package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServo;
import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServoPair;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

@Config
public class Lift extends SubsystemBase {
    // Config parameters
    public static TrapezoidProfile.Constraints liftConstraints = new TrapezoidProfile.Constraints(2500, 2500);
    public static PIDCoefficients liftCoefficients = new PIDCoefficients(0.005, 0, 0);
    public static ProfiledPIDController liftController = new ProfiledPIDController(
            liftCoefficients.kP,
            liftCoefficients.kI,
            liftCoefficients.kD,
            liftConstraints
    );
    public static int liftLow = 625;
    public static int liftMid = 1250;
    public static int liftHigh = 1875;
    public static int dropDownDistance = 160;
    public static double grabPos = 0.9;
    public static double releasePos = 0.65;
    public static double waitTime = 1.5;
    public static int hoverPos = 650;
    public static int collectPos = 100;
    public static int rotationLow = 500;
    public static int rotationMid = 1000;
    public static int rotationHigh = 1500;
    public static int errorTolerance = 10;
    public static double grabTime = 0.75;
    public static double yawArmAngle = -5;
    public static double yawArmRetracted = 0.65;
    public static double yawArmExtended = 0.8;
    public static double depositThreshold = 0.75;
    public static double jamThreshold = 10;

    private int rotationHeight = rotationLow;

    private final Telemetry telemetry;

    private final DcMotorEx lift1;
    private final DcMotorEx lift2;
    private final ProfiledServoPair yawArm;
    private final Servo grabber;
    private final ProfiledServo yawArmExtension;

    private int targetPosition = 0;
    private int previousTargetPosition = 0;
    private int error1 = 0;
    private int error2 = 0;

    private double currentYawArmAngle = 0;

    public enum LiftState {
        HIGH,
        MID,
        LOW,
        LOWERED,
        GOING_UP,
        RETRACT,
        COLLECT,
        ZERO
    }

    private LiftState liftState = LiftState.RETRACT;
    private LiftState previousLiftState = LiftState.HIGH;

    private enum GrabberState {
        HOLD,
        RELEASE,
        DEPOSITING
    }

    private GrabberState grabberState = GrabberState.HOLD;

    public enum YawArmState {
        RETRACTED,
        EXTENDED
    }

    private YawArmState yawArmState = YawArmState.RETRACTED;

    private final ElapsedTime grabTimer = new ElapsedTime();

    public Lift(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        yawArm = new ProfiledServoPair(
                hardwareMap,
                "yaw1",
                "yaw2",
                new MotionConstraint(6, 8, 6),
                (0.0037037 * -45) + 0.33333
        );

        grabber = hardwareMap.get(Servo.class, "grab");
        grabber.setPosition(grabPos);

        yawArmExtension = new ProfiledServo(
                hardwareMap,
                "extension",
                new MotionConstraint(2, 4, 4),
                yawArmRetracted
        );

        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setLiftState(LiftState state) {
        liftState = state;
    }

    public LiftState getLiftState() {
        return liftState;
    }

    public LiftState getPreviousLiftState() {
        return previousLiftState;
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
    public double getYawArmAngle() {
        return currentYawArmAngle;
    }

    public int getSlidePosition() { return lift1.getCurrentPosition(); }

    public double[] getMotorPowers() { return new double[]{lift1.getPower(), lift2.getPower()}; }

    public int getTargetPosition() { return targetPosition; }

    public boolean canControlArm() {
        return lift1.getCurrentPosition() > rotationHeight;
    }

    public boolean isBusy() { return Math.abs(targetPosition - lift1.getCurrentPosition()) >= errorTolerance; }

    public boolean isYawArmBusy() { return yawArm.isBusy(); }

    public void setYawArmExtensionState(YawArmState state) {
        yawArmState = state;
    }

    public boolean isJammed() {
        return lift1.getCurrent(CurrentUnit.AMPS) > jamThreshold;
    }

    @Override
    public void periodic() {
        switch (liftState) {
            case HIGH:
                targetPosition = liftHigh;
                previousTargetPosition = liftHigh;
                rotationHeight = rotationHigh;
                previousLiftState = LiftState.HIGH;
                setYawArmExtensionState(YawArmState.EXTENDED);
                break;
            case MID:
                targetPosition = liftMid;
                previousTargetPosition = liftMid;
                rotationHeight = rotationMid;
                previousLiftState = LiftState.MID;
                setYawArmExtensionState(YawArmState.EXTENDED);
                break;
            case LOW:
                targetPosition = liftLow;
                previousTargetPosition = liftLow;
                rotationHeight = rotationLow;
                previousLiftState = LiftState.LOW;
                setYawArmExtensionState(YawArmState.EXTENDED);
                break;
            case LOWERED:
                targetPosition = previousTargetPosition - dropDownDistance;
                break;
            case GOING_UP:
                targetPosition = hoverPos;
                grabberState = GrabberState.HOLD;
                if (!isBusy()) {
                    setYawArmAngle(yawArmAngle);
                    setYawArmExtensionState(YawArmState.RETRACTED);
                }
                break;
            case RETRACT:
                setYawArmAngle(yawArmAngle);
                setYawArmExtensionState(YawArmState.RETRACTED);
                grabberState = GrabberState.HOLD;
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

        lift1.setPower(liftController.calculate(lift1.getCurrentPosition(), targetPosition));
        lift2.setPower(liftController.calculate(lift2.getCurrentPosition(), targetPosition));

        if (isJammed()) {
            setLiftState(LiftState.RETRACT);
        }

        switch (grabberState) {
            case HOLD:
                grabber.setPosition(grabPos);
                break;
            case RELEASE:
                grabber.setPosition(releasePos);
                break;
        }

        switch (yawArmState) {
            case RETRACTED:
                yawArmExtension.setPosition(yawArmRetracted);
                break;

            case EXTENDED:
                yawArmExtension.setPosition(yawArmExtended);
                break;
        }

        yawArm.periodic();
        yawArmExtension.periodic();
    }
}
