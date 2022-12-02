package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ScoringMech {
    // Config parameters
    public static double clawOpenPos = 0;
    public static double clawClosedPos = 1;
    public static double v4b1RetractedPos = 0;
    public static double v4b1ExtendedPos = 1;
    public static double v4b2RetractedPos = 1;
    public static double v4b2ExtendedPos = 0;
    public static double desiredSlidePower = 0.5;
    public static double maxExtension = 1500;
    public static double grabTime = 1;

    public static int liftLow = 330;
    public static int liftMid = 515;
    public static int liftHigh = 710;
    public static double grabPos = 0.5;
    public static double releasePos = 0.9;
    public static double waitTime = 1.5;
    public static double desiredLiftPower = 0.5;
    public static int liftCollectPos = 150;
    public static int liftGrabPos = 0;
    public static double yawArm1Default = 0;
    public static double yawArm2Default = 1;

    public enum ScoringState {
        RETRACTED,
        EXTENDING,
        EXTENDED,
        GRABBING,
        RETRACTING,
        TRANSFERRING,
        LOWERED,
        LIFTING,
        CONTROLLING_ARM,
        DEPOSITING,
        LOWERING,
        RESET
    }

    public ScoringState scoringState;

    public final Telemetry telemetry;

    public final DcMotorEx lift1;
    public final DcMotorEx lift2;
    public final ServoImplEx yawArm1;
    public final ServoImplEx yawArm2;
    public final ServoImplEx grabber;

    public DcMotorEx slide1;
    public DcMotorEx slide2;
    public ServoImplEx claw;
    public ServoImplEx v4b1;
    public ServoImplEx v4b2;

    public final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public double liftPower;
    public int depositTicks;

    public boolean grabbing = false;
    public boolean previousIntakeGrabButton = false;
    public boolean previousTransferButton = false;

    public ScoringMech(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        slide1 = hardwareMap.get(DcMotorEx.class, "intake1");
        slide2 = hardwareMap.get(DcMotorEx.class, "intake2");
        v4b1 = hardwareMap.get(ServoImplEx.class, "v4b1");
        v4b2 = hardwareMap.get(ServoImplEx.class, "v4b2");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        yawArm1 = hardwareMap.get(ServoImplEx.class, "yaw1");
        yawArm2 = hardwareMap.get(ServoImplEx.class, "yaw2");
        grabber = hardwareMap.get(ServoImplEx.class, "grab");

        slide1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        v4b1.setPosition(v4b1RetractedPos);
        v4b2.setPosition(v4b2RetractedPos);
        claw.setPosition(clawOpenPos);

        yawArm1.setPosition(yawArm1Default);
        yawArm2.setPosition(yawArm2Default);
        grabber.setPosition(grabPos);
    }

    public void score(double intakeExtension, double intakeRetraction, boolean intakeGrabButton, boolean automaticIntakeRetraction,
                      boolean liftButtonHigh, boolean liftButtonMid, boolean liftButtonLow,
                     boolean transferButton, boolean depositButton, boolean cancelAutomation) {
        telemetry.addData("Scoring State", scoringState);
        telemetry.addData("Lift Power", liftPower);
        telemetry.addData("Lift Encoder Value", lift1.getCurrentPosition());
        telemetry.addData("Lift Target Position", lift1.getTargetPosition());
        telemetry.addData("Grab Servo Pos", grabber.getPosition());
        telemetry.addData("Timer", eTime.time());
        telemetry.update();

        switch (scoringState) {
            case RETRACTED:
                slide1.setTargetPosition(0);
                slide2.setTargetPosition(0);
                v4b1.setPosition(v4b1RetractedPos);
                v4b2.setPosition(v4b2RetractedPos);
                claw.setPosition(clawOpenPos);
                if (intakeExtension > 0) {
                    slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    scoringState = ScoringState.EXTENDING;
                }
                break;

            case EXTENDING:
                claw.setPosition(clawOpenPos);
                slide1.setPower(intakeExtension - intakeRetraction);
                slide2.setPower(intakeExtension - intakeRetraction);
                v4b1.setPosition(v4b1ExtendedPos);
                v4b2.setPosition(v4b2ExtendedPos);
                if (intakeGrabButton) {
                    scoringState = ScoringState.GRABBING;
                }
                break;

            case GRABBING:
                claw.setPosition(clawClosedPos);
                if (automaticIntakeRetraction) {
                    scoringState = ScoringState.RETRACTING;
                }
                if (intakeGrabButton && !previousIntakeGrabButton) {
                    scoringState = ScoringState.EXTENDING;
                }
                break;

            case RETRACTING:
                slide1.setTargetPosition(0);
                slide2.setTargetPosition(0);
                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                v4b1.setPosition(v4b1RetractedPos);
                v4b2.setPosition(v4b2RetractedPos);
                if (Math.abs(slide1.getCurrentPosition()) < 20) {
                    scoringState = ScoringState.LOWERED;
                }
                break;
            case TRANSFERRING:
                
            case LOWERED:
                if (liftButtonHigh) {
                    depositTicks = liftHigh;
                    lift1.setTargetPosition(depositTicks);
                    lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    lift2.setTargetPosition(depositTicks);
                    lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    scoringState = ScoringState.LIFTING;
                } else if (liftButtonMid) {
                    depositTicks = liftMid;
                    lift1.setTargetPosition(depositTicks);
                    lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    lift2.setTargetPosition(depositTicks);
                    lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    scoringState = ScoringState.LIFTING;
                } else if (liftButtonLow) {
                    depositTicks = liftLow;
                    lift1.setTargetPosition(depositTicks);
                    lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    lift2.setTargetPosition(depositTicks);
                    lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    scoringState = ScoringState.LIFTING;
                } else if (transferButton && !previousTransferButton) {
                    if (grabbing) {
                        depositTicks = liftCollectPos;
                        grabbing = false;
                    } else {
                        depositTicks = liftGrabPos;
                        grabbing = true;
                    }
                    lift1.setTargetPosition(depositTicks);
                    lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    lift2.setTargetPosition(depositTicks);
                    lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                }
                previousTransferButton = transferButton;
                break;

            case LIFTING:
                lift1.setPower(desiredLiftPower);
                lift2.setPower(desiredLiftPower);
                grabber.setPosition(grabPos);
                if (lift1.getCurrentPosition() > minHeightForExtension) {
                    liftState = LiftState.EXTEND;
                }
                break;

            case EXTEND:
                horizontal1.setPosition(h1Extended);
                horizontal2.setPosition(h2Extended);
                grabber.setPosition(grabPos);
                if (Math.abs(lift1.getCurrentPosition() - depositTicks) < 20) {
                    if (depositButton) {
                        liftPower = 0;
                        eTime.reset();
                        liftState = LiftState.DEPOSIT;
                    }
                }
                break;

            case DEPOSIT:
                grabber.setPosition(releasePos);
                if (eTime.time() > waitTime) {
                    grabber.setPosition(grabPos);
                    liftState = LiftState.RETRACT;
                }
                break;

            case RETRACT:
                lift1.setTargetPosition(liftCollectPos);
                lift2.setTargetPosition(liftCollectPos);
                liftPower = desiredLiftPower;
                horizontal1.setPosition(h1Retracted);
                horizontal2.setPosition(h2Retracted);
                if (Math.abs(liftCollectPos - lift1.getCurrentPosition()) < 10) {
                    liftState = LiftState.STOP;
                }
                break;

            case RESET:
                depositTicks = liftCollectPos;
                lift1.setTargetPosition(depositTicks);
                lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                lift2.setTargetPosition(depositTicks);
                lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                grabbing = false;
                liftState = LiftState.START;
        }

        if (cancelAutomation) {
            scoringState = ScoringState.STOP;
        }

        previousIntakeGrabButton = intakeGrabButton;
    }

    public void extendHigh() {
        lift1.setTargetPosition(liftHigh);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(liftHigh);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
        horizontal1.setPosition(h1Extended);
        horizontal2.setPosition(h2Extended);
    }

    public void extendMid() {
        lift1.setTargetPosition(liftMid);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(liftMid);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
        horizontal1.setPosition(h1Extended);
        horizontal2.setPosition(h2Extended);
    }

    public void extendLow() {
        lift1.setTargetPosition(liftLow);
        lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift1.setPower(liftPower);
        lift2.setTargetPosition(liftLow);
        lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift2.setPower(liftPower);
        horizontal1.setPosition(h1Extended);
        horizontal2.setPosition(h2Extended);
    }

    public void deposit() {
        grabber.setPosition(releasePos);
    }

    public void retract() {
        grabber.setPosition(grabPos);
        horizontal1.setPosition(h1Retracted);
        horizontal2.setPosition(h2Retracted);
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
}