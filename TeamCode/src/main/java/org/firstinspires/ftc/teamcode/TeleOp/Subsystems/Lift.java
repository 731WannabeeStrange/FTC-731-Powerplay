package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {
    // Config parameters
    public static double h1Retracted = 1;
    public static double h2Retracted = 0;
    public static double h1Extended = 0.3;
    public static double h2Extended = 0.7;
    public static int liftLow = 1600;
    public static int liftMid = 2600;
    public static int liftHigh = 3600;
    public static double grabPos = 0.4;
    public static double releasePos = 0.25;
    public static double waitTime = 0.5;
    public static double desiredLiftPower = 1;
    public static double minHeightForExtension = 600;
    public static int liftCollectPos = 300;
    public static int liftGrabPos = 0;

    public enum LiftState {
        START,
        LIFT,
        EXTEND,
        DEPOSIT,
        RETRACT,
        STOP
    }

    public LiftState liftState = LiftState.START;

    public final Telemetry telemetry;

    public final DcMotorEx lift;
    public final ServoImplEx horizontal1;
    public final ServoImplEx horizontal2;
    public final ServoImplEx grabber;

    public final ElapsedTime eTime = new ElapsedTime();

    public double liftPower;
    public int depositTicks;

    public Lift(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        horizontal1 = hardwareMap.get(ServoImplEx.class, "h1");
        horizontal2 = hardwareMap.get(ServoImplEx.class, "h2");
        grabber = hardwareMap.get(ServoImplEx.class, "grab");

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        horizontal1.setPosition(h1Retracted);
        horizontal2.setPosition(h2Retracted);
    }

    public void lift(boolean liftButtonHigh, boolean liftButtonMid, boolean liftButtonLow,
                     boolean grabButton, boolean depositButton, boolean cancelAutomation) {
        telemetry.addData("Lift State", liftState);
        telemetry.addData("Lift Power", liftPower);
        telemetry.addData("Lift Encoder Value", lift.getCurrentPosition());
        telemetry.addData("Lift Target Position", lift.getTargetPosition());
        telemetry.update();

        switch (liftState) {
            case START:
                if (liftButtonHigh) {
                    depositTicks = liftHigh;
                    lift.setTargetPosition(depositTicks);
                    lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    liftState = LiftState.LIFT;
                } else if (liftButtonMid) {
                    depositTicks = liftMid;
                    lift.setTargetPosition(depositTicks);
                    lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    liftState = LiftState.LIFT;
                } else if (liftButtonLow) {
                    depositTicks = liftLow;
                    lift.setTargetPosition(depositTicks);
                    lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    liftState = LiftState.LIFT;
                } else if (grabButton) {
                    if (depositTicks == liftGrabPos) {
                        depositTicks = liftCollectPos;
                    } else {
                        depositTicks = liftGrabPos;
                    }
                    lift.setTargetPosition(depositTicks);
                    lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                }
                break;

            case LIFT:
                lift.setPower(desiredLiftPower);
                if (lift.getCurrentPosition() > minHeightForExtension) {
                    liftState = LiftState.EXTEND;
                }
                break;

            case EXTEND:
                horizontal1.setPosition(h1Extended);
                horizontal2.setPosition(h2Extended);
                if (Math.abs(lift.getCurrentPosition() - depositTicks) < 20) {
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
                lift.setTargetPosition(liftCollectPos);
                liftPower = desiredLiftPower;
                if (Math.abs(liftCollectPos - lift.getCurrentPosition()) < 10) {
                    liftState = LiftState.STOP;
                }
                break;

            case STOP:
                depositTicks = liftCollectPos;
                lift.setTargetPosition(depositTicks);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                liftState = LiftState.START;
        }

        if (cancelAutomation) {
            liftState = LiftState.STOP;
        }
    }

    public void extendHigh() {
        lift.setTargetPosition(liftHigh);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(liftPower);
        horizontal1.setPosition(h1Extended);
        horizontal2.setPosition(h2Extended);
    }

    public void extendMid() {
        lift.setTargetPosition(liftMid);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(liftPower);
        horizontal1.setPosition(h1Extended);
        horizontal2.setPosition(h2Extended);
    }

    public void extendLow() {
        lift.setTargetPosition(liftLow);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(liftPower);
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
        lift.setTargetPosition(liftCollectPos);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(liftPower);
    }

    public void grab() {
        lift.setTargetPosition(liftGrabPos);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(liftPower);
    }
}