package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        LOWERED,
        LIFTING,
        CONTROLLING_ARM,
        DEPOSITING,
        LOWERING,
        RESET
    }

    public ScoringState scoringState = ScoringState.RESET;

    Lift lift;
    Intake intake;

    MultipleTelemetry telemetry;

    public final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public boolean previousIntakeGrabButton = false;

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
                if (intakeGrabButton) {
                    intake.setV4bPos(1.5);
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
                    intake.retractFully();
                    scoringState = ScoringState.RETRACTING;
                }
                break;

            case RETRACTING:
                if (!intake.isBusy()) {
                    intake.setV4bPos(0.6);
                    eTime.reset();
                    scoringState = ScoringState.RAISINGV4B;
                }
                break;

            case RAISINGV4B:
                if (eTime.time() > 0.5) {
                    scoringState = ScoringState.TRANSFERRING;
                }
                break;

            case TRANSFERRING:
                lift.collect();
                if (!lift.isBusy()) {
                    lift.grab();
                    intake.release();
                    scoringState = ScoringState.LOWERED;
                }
                break;

            case LOWERED:
                if (liftButtonHigh) {
                    lift.extendHigh();
                    scoringState = ScoringState.LIFTING;
                } else if (liftButtonMid) {
                    lift.extendMid();
                    scoringState = ScoringState.LIFTING;
                } else if (liftButtonLow) {
                    lift.extendLow();
                    scoringState = ScoringState.LIFTING;
                }
                break;

            case LIFTING:
                if (lift.getSlidePosition() > Lift.minHeightForArmRotation) {
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

                if (!lift.isBusy()) {
                    if (depositButton) {
                        lift.deposit();
                        eTime.reset();
                        scoringState = ScoringState.DEPOSITING;
                    }
                }
                break;

            case DEPOSITING:
                if (eTime.time() > Lift.waitTime) {
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
                intake.retractFully();
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
}