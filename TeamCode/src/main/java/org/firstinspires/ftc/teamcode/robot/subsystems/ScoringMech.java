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

    Lift lift;
    Intake intake;

    MultipleTelemetry telemetry;

    public final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public boolean previousIntakeGrabButton = false;

    public ScoringMech(HardwareMap hardwareMap, MultipleTelemetry multipleTelemetry) {
        lift = new Lift(hardwareMap, multipleTelemetry);
        intake = new Intake(hardwareMap, multipleTelemetry);
    }

    public void score(double intakeExtension, double intakeRetraction, boolean intakeGrabButton, boolean automaticIntakeRetraction,
                      boolean liftButtonHigh, boolean liftButtonMid, boolean liftButtonLow,
                      boolean depositButton, double yawArmY, double yawArmX, boolean cancelAutomation,
                      boolean yawArm0, boolean yawArm90, boolean yawArm180, boolean yawArm270) {
        telemetry.addData("Timer", eTime.time());
        telemetry.update();

        switch (scoringState) {
            case RETRACTED:
                intake.retractFully();
                if (intakeExtension > 0) {
                    intake.slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intake.slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    scoringState = ScoringState.EXTENDING;
                }
                break;

            case EXTENDING:
                intake.extend(intakeExtension, intakeRetraction);
                if (intakeGrabButton) {
                    scoringState = ScoringState.GRABBING;
                }
                break;

            case GRABBING:
                intake.grab();
                if (automaticIntakeRetraction) {
                    scoringState = ScoringState.RETRACTING;
                }
                if (intakeGrabButton && !previousIntakeGrabButton) {
                    scoringState = ScoringState.EXTENDING;
                }
                break;

            case RETRACTING:
                intake.retractFully();
                if (Math.abs(intake.getSlidePosition()) < 5) {
                    scoringState = ScoringState.TRANSFERRING;
                }
                break;
            case TRANSFERRING:
                lift.grab();
                if (Math.abs(lift.getSlidePosition() - Lift.liftGrabPos) < 5){
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
                lift.grab();
                if (lift.getSlidePosition() > Lift.minHeightForArmRotation) {
                    scoringState = ScoringState.CONTROLLING_ARM;
                }
                break;

            case CONTROLLING_ARM:
                lift.setYawArmAngle(Math.atan2(yawArmY, yawArmX));
                if (yawArm0) {
                    lift.setYawArmAngle(0);
                } else if (yawArm90) {
                    lift.setYawArmAngle(90);
                } else if (yawArm180) {
                    lift.setYawArmAngle(180);
                } else if (yawArm270) {
                    lift.setYawArmAngle(270);
                }

                if (Math.abs(lift.getSlidePosition() - lift.getTargetPosition()) < 5) {
                    if (depositButton) {
                        eTime.reset();
                        scoringState = ScoringState.DEPOSITING;
                    }
                }
                break;

            case DEPOSITING:
                lift.deposit();
                if (eTime.time() > Lift.waitTime) {
                    lift.grab();
                    scoringState = ScoringState.LOWERING;
                }
                break;

            case LOWERING:
                lift.retract();
                if (Math.abs(lift.getSlidePosition() - lift.getTargetPosition()) < 5) {
                    scoringState = ScoringState.RETRACTED;
                }
                break;

            case RESET:
                intake.retractFully();
                lift.retract();
                break;
        }

        if (cancelAutomation) {
            scoringState = ScoringState.RESET;
        }

        previousIntakeGrabButton = intakeGrabButton;
    }
}