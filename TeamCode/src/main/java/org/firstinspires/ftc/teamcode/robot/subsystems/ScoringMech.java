package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.LiftState;

@Config
public class ScoringMech {
    public enum ScoringState {
        RETRACTED,
        DROPPINGV4B,
        EXTENDING,
        GRABBING,
        RETRACTING,
        RAISINGV4B,
        TRANSFERRING,
        RELEASING,
        LOWERED,
        LIFTING,
        CONTROLLING_ARM,
        DEPOSITING,
        LOWERING,
        RESET
    }

    public ScoringState scoringState = ScoringState.RESET;

    private LiftState previousLiftState = LiftState.HIGH;

    private boolean controllingArm = false;

    private Lift lift;
    private Intake intake;

    private MultipleTelemetry telemetry;

    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private boolean previousIntakeGrabButton = false;
    private double previousYawArmAngle = 0;

    public ScoringMech(HardwareMap hardwareMap, MultipleTelemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        lift = new Lift(hardwareMap, multipleTelemetry);
        intake = new Intake(hardwareMap, multipleTelemetry);
    }

    public void score(boolean intakeGrabButton, boolean liftButtonHigh, boolean liftButtonMid, boolean liftButtonLow,
                      boolean depositButton, double yawArmY, double yawArmX, boolean cancelAutomation,
                      boolean yawArm0, boolean yawArm90, boolean yawArm180, boolean yawArm270) {
        telemetry.addData("Timer", eTime.time());
        telemetry.addData("smState", scoringState);
        telemetry.update();

        switch (scoringState) {
            case RETRACTED:
                intake.retractFully();
                intake.grab();
                lift.grab();
                if (intakeGrabButton) {
                    intake.setV4bPos(0.9);
                    scoringState = ScoringState.DROPPINGV4B;
                }
                break;

            case DROPPINGV4B:
                if (!intake.isV4BBusy()) {
                    intake.release();
                    intake.extendFully();
                    scoringState = ScoringState.EXTENDING;
                }
                break;
                
            case EXTENDING:
                if (intake.isConeDetected()) {
                    intake.stopSlides();
                    intake.grab();
                    scoringState = ScoringState.GRABBING;
                }
                if (!intake.isBusy() && !intake.isConeDetected()) {
                    scoringState = ScoringState.RESET;
                }
                break;

            case GRABBING:
                if (!intake.isClawBusy()) {
                    intake.retractPart(0.13);
                    scoringState = ScoringState.RETRACTING;
                }
                break;

            case RETRACTING:
                if (!intake.isBusy() && !intake.isV4BBusy()) {
                    lift.deposit();
                    lift.collect();
                    scoringState = ScoringState.TRANSFERRING;
                }
                break;

            case TRANSFERRING:
                if (!lift.isBusy()) {
                    lift.grab();
                    intake.release();
                    scoringState = ScoringState.RELEASING;
                }
                break;

            case RELEASING:
                if (!intake.isClawBusy()) {
                    intake.retractFully();
                    scoringState = ScoringState.LOWERED;
                }
                break;

            case LOWERED:
                if (!intake.isBusy()) {
                    switch (previousLiftState) {
                        case HIGH:
                            lift.extendHigh();
                            scoringState = ScoringState.LIFTING;
                            break;
                        case MID:
                            lift.extendMid();
                            scoringState = ScoringState.LIFTING;
                            break;
                        case LOW:
                            lift.extendLow();
                            scoringState = ScoringState.LIFTING;
                            break;
                        default:
                            throw new IllegalStateException("Unexpected previous lift state: " + previousLiftState);
                    }
                }
                break;

            case LIFTING:
                if (lift.canControlArm()) {
                    controllingArm = true;
                    lift.setYawArmAngle(previousYawArmAngle);
                    scoringState = ScoringState.CONTROLLING_ARM;
                }
                break;

            case CONTROLLING_ARM:
                if (yawArmY != 0 || yawArmX != 0) {
                    lift.setYawArmAngle(Math.atan2(yawArmY, yawArmX));
                }
                if (yawArm0) {
                    lift.setYawArmAngle(0);
                } else if (yawArm90) {
                    lift.setYawArmAngle(90);
                } else if (yawArm180) {
                    lift.setYawArmAngle(180);
                } else if (yawArm270) {
                    lift.setYawArmAngle(270);
                }

                if (liftButtonHigh) {
                    lift.extendHigh();
                } else if (liftButtonMid) {
                    lift.extendMid();
                } else if (liftButtonLow) {
                    lift.extendLow();
                }

                if (!lift.isBusy()) {
                    if (depositButton) {
                        previousYawArmAngle = lift.getYawArmAngle();
                        previousLiftState = lift.getLiftState();

                        lift.deposit();
                        eTime.reset();
                        scoringState = ScoringState.DEPOSITING;
                    }
                }
                break;

            case DEPOSITING:
                if (eTime.time() > Lift.waitTime) {
                    controllingArm = false;
                    lift.grab();
                    scoringState = ScoringState.LOWERING;
                }
                break;

            case LOWERING:
                lift.retract();
                if (!lift.isBusy()) {
                    scoringState = ScoringState.RETRACTED;
                }
                break;

            case RESET:
                controllingArm = false;
                intake.retractFully();
                intake.grab();
                lift.retract();
                scoringState = ScoringState.RETRACTED;
                break;
        }

        lift.update();
        intake.update();

        if (cancelAutomation) {
            scoringState = ScoringState.RESET;
        }

        previousIntakeGrabButton = intakeGrabButton;
    }

    public boolean isControllingArm() {
        return controllingArm;
    }
}