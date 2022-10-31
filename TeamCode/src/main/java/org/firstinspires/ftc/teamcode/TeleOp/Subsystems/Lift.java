package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {
    // Config parameters
    public static double h1Retracted = 0.65;
    public static double h2Retracted = 0;
    public static double h1Extended = 0.3;
    public static double h2Extended = 0.7;
    public static int liftLow = 330;
    public static int liftMid = 515;
    public static int liftHigh = 710;
    public static double grabPos = 0.5;
    public static double releasePos = 0.9;
    public static double waitTime = 1.5;
    public static double desiredLiftPower = 0.5;
    public static double minHeightForExtension = 250;
    public static int liftCollectPos = 150;
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

    public final DcMotorEx lift1;
    public final DcMotorEx lift2;
    public final ServoImplEx horizontal1;
    public final ServoImplEx horizontal2;
    public final ServoImplEx grabber;

    public final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public double liftPower;
    public int depositTicks;

    public boolean grabbing = false;
    public boolean previousGrabButton = false;

    public Lift(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        horizontal1 = hardwareMap.get(ServoImplEx.class, "h1");
        horizontal2 = hardwareMap.get(ServoImplEx.class, "h2");
        grabber = hardwareMap.get(ServoImplEx.class, "grab");

        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        horizontal1.setPosition(h1Retracted);
        horizontal2.setPosition(h2Retracted);

        grabber.setPosition(grabPos);
    }

    public void lift(boolean liftButtonHigh, boolean liftButtonMid, boolean liftButtonLow,
                     boolean grabButton, boolean depositButton, boolean cancelAutomation) {
        telemetry.addData("Lift State", liftState);
        telemetry.addData("Lift Power", liftPower);
        telemetry.addData("Lift Encoder Value", lift1.getCurrentPosition());
        telemetry.addData("Lift Target Position", lift1.getTargetPosition());
        telemetry.addData("Grab Servo Pos", grabber.getPosition());
        telemetry.addData("Timer", eTime.time());
        telemetry.update();

        switch (liftState) {
            case START:
                if (liftButtonHigh) {
                    depositTicks = liftHigh;
                    lift1.setTargetPosition(depositTicks);
                    lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    lift2.setTargetPosition(depositTicks);
                    lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    liftState = LiftState.LIFT;
                } else if (liftButtonMid) {
                    depositTicks = liftMid;
                    lift1.setTargetPosition(depositTicks);
                    lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    lift2.setTargetPosition(depositTicks);
                    lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    liftState = LiftState.LIFT;
                } else if (liftButtonLow) {
                    depositTicks = liftLow;
                    lift1.setTargetPosition(depositTicks);
                    lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    lift2.setTargetPosition(depositTicks);
                    lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    liftState = LiftState.LIFT;
                } else if (grabButton && !previousGrabButton) {
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
                previousGrabButton = grabButton;
                break;

            case LIFT:
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

            case STOP:
                depositTicks = liftCollectPos;
                lift1.setTargetPosition(depositTicks);
                lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                lift2.setTargetPosition(depositTicks);
                lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                grabbing = false;
                liftState = LiftState.START;
        }

        if (cancelAutomation) {
            liftState = LiftState.STOP;
        }
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