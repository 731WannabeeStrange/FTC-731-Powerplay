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
        LOWERED,
        COLLECTING,
        LIFTING,
        CONTROLLING_ARM,
        DEPOSITING,
        LOWERING,
    }

    public ScoringState scoringState;

    Lift lift;

    MultipleTelemetry telemetry;

    public final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public ScoringMech(HardwareMap hardwareMap, MultipleTelemetry multipleTelemetry) {
        lift = new Lift(hardwareMap, multipleTelemetry);
    }

    public void score(boolean intakeExtension, boolean intakeRetraction, boolean liftGrabButton,
                      boolean liftButtonHigh, boolean liftButtonMid, boolean liftButtonLow,
                      boolean depositButton, double yawArmY, double yawArmX, boolean cancelAutomation,
                      boolean yawArm0, boolean yawArm90, boolean yawArm180, boolean yawArm270) {
        telemetry.addData("Timer", eTime.time());
        telemetry.update();

        switch (scoringState) {
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
                } else if (liftGrabButton) {
                    lift.collect();
                    scoringState = ScoringState.COLLECTING;
                }
                break;
            case COLLECTING:
                if (liftButtonHigh) {
                    lift.extendHigh();
                    scoringState = ScoringState.LIFTING;
                } else if (liftButtonMid) {
                    lift.extendMid();
                    scoringState = ScoringState.LIFTING;
                } else if (liftButtonLow) {
                    lift.extendLow();
                    scoringState = ScoringState.LIFTING;
                } else if (liftGrabButton) {
                    lift.retract();
                    scoringState = ScoringState.LOWERED;
                }
                break;

            case LIFTING:
                lift.grab();
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
                    scoringState = ScoringState.LOWERED;
                }
                break;
        }

        if (cancelAutomation) {
            scoringState = ScoringState.LOWERING;
        }
    }
}