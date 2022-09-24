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
    public static double h1Original = 1;
    public static double h2Original = 0;
    public static double h1Extended = 0.3;
    public static double h2Extended = 0.7;
    public static double liftLow = 600;
    public static double liftMid = 1600;
    public static double liftHigh = 2600;
    public static double grabPos = 1;
    public static double releasePos = 0.5;
    public static double waitTime = 0.5;

    private enum LiftState {
        START,
        LIFT,
        EXTEND,
        DEPOSIT,
        RETRACT,
        STOP
    }

    private LiftState liftState = LiftState.START;

    private final Telemetry telemetry;

    private final DcMotorEx lift;
    private final ServoImplEx horizontal1;
    private final ServoImplEx horizontal2;
    private final ServoImplEx grabber;

    private final ElapsedTime eTime = new ElapsedTime();

    public Lift(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        horizontal1 = hardwareMap.get(ServoImplEx.class, "h1");
        horizontal2 = hardwareMap.get(ServoImplEx.class, "h2");
        grabber = hardwareMap.get(ServoImplEx.class, "grab");

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void init() {
        horizontal1.setPosition(h1Original);
        horizontal2.setPosition(h2Original);

    }

    public void lift() {
        telemetry.addData("Lift State", liftState);
    }
}
