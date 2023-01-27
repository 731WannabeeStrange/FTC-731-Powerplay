package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group="test")
@Config
public class VerticalTestOpMode extends LinearOpMode
{
    private Servo s1;
    private Servo s2;

    public static double retractedPos = 1.0;
    public static double extendedPos = 0.4;
    private double currentPos = 1;

    private boolean lbTriggered = false, rbTriggered = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && !lbTriggered) {
                currentPos = extendedPos;
            } else if (gamepad1.right_bumper && !rbTriggered) {
                currentPos = retractedPos;
            }

            lbTriggered = gamepad1.left_bumper;
            rbTriggered = gamepad1.right_bumper;

            s1.setPosition(currentPos);

            telemetry.addData("s position", currentPos);

            telemetry.update();
        }
    }
}