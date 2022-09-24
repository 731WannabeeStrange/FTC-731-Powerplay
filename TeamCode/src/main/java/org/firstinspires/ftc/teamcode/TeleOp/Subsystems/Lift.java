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
    public static int liftLow = 600;
    public static int liftMid = 1600;
    public static int liftHigh = 2600;
    public static double grabPos = 1;
    public static double releasePos = 0.5;
    public static double waitTime = 0.5;
    public static double desiredLiftPower = 1;
    public static double minHeightForExtension = 400;

    private enum LiftState {
        START,
        LIFT,
        EXTEND,
        DEPOSIT,
        RETRACT,
        STOP
    }

    private LiftState liftState = LiftState.START;

    /*
    private enum DepositLocation {
        LOW(liftLow),
        MID(liftMid),
        HIGH(liftHigh);

        private static final DepositLocation[] states = values();

        public int ticks;

        DepositLocation(int ticks) {
            this.ticks = ticks;
        }
    }

    private DepositLocation depositLocation = DepositLocation.HIGH;
    */

    private final Telemetry telemetry;

    private final DcMotorEx lift;
    private final ServoImplEx horizontal1;
    private final ServoImplEx horizontal2;
    private final ServoImplEx grabber;

    private final ElapsedTime eTime = new ElapsedTime();

    private double liftPower;
    private int depositTicks;

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
                     boolean depositButton, boolean cancelAutomation) {
        telemetry.addData("Lift State", liftState);
        telemetry.addData("Lift Power", liftPower);
        telemetry.addData("Lift Encoder Value", lift.getCurrentPosition());
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
                }
                break;

            case LIFT:
                liftPower = desiredLiftPower;
                if (lift.getCurrentPosition() > minHeightForExtension) {
                    liftState = LiftState.EXTEND;
                }
                break;

            case EXTEND:
                horizontal1.setPosition(h1Extended);
                horizontal2.setPosition(h2Extended);
                if (lift.getCurrentPosition() - depositTicks < 20) {
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
                lift.setTargetPosition(0);
                liftPower = desiredLiftPower;
                if (lift.getCurrentPosition() < 10) {
                    liftState = LiftState.STOP;
                }
                break;

            case STOP:
                liftPower = 0;
                liftState = LiftState.START;
                break;
        }

        if (cancelAutomation) {
            liftState = LiftState.STOP;
        }
    }
}
