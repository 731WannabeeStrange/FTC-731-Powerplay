package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    // Config parameters
    public static double clawOpenPos = 0;
    public static double clawClosedPos = 1;
    public static double v4b1RetractedPos = 0;
    public static double v4b1ExtendedPos = 1;
    public static double v4b2RetractedPos = 1;
    public static double v4b2ExtendedPos = 0;
    public static double retractTime = 0.5;
    public static int[][] stackPositions = {
            {0, 0},
            {0, 0},
            {0, 0},
            {0, 0},
            {0, 0}
    };

    public final Telemetry telemetry;

    public ServoImplEx claw;
    public ServoImplEx v4b1;
    public ServoImplEx v4b2;

    public final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public int depositTicks;

    public boolean grabbing = false;
    public boolean previousGrabButton = false;

    public Intake(HardwareMap hardwareMap, Telemetry multipleTelemetry) {
        telemetry = multipleTelemetry;

        v4b1 = hardwareMap.get(ServoImplEx.class, "v4b1");
        v4b2 = hardwareMap.get(ServoImplEx.class, "v4b2");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        v4b1.setPosition(v4b1RetractedPos);
        v4b2.setPosition(v4b2RetractedPos);

        claw.setPosition(clawOpenPos);

    }

    public void grab() {
        claw.setPosition(clawClosedPos);
    }

    public void release() {
        claw.setPosition(clawOpenPos);
    }

    public void setV4BPositions(double p1, double p2) {
        v4b1.setPosition(p1);
        v4b2.setPosition(p2);
    }

    public void extendV4B() {
        v4b1.setPosition(v4b1ExtendedPos);
        v4b2.setPosition(v4b2ExtendedPos);
    }

    public void retractV4B() {
        v4b1.setPosition(v4b1RetractedPos);
        v4b2.setPosition(v4b2RetractedPos);
    }
}