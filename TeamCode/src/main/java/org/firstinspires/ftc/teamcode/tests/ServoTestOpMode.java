package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group="test")
@Config
public class ServoTestOpMode extends LinearOpMode
{
    private Servo s1;
    private Servo s2;

    private double s1pos = 0.5;
    private double s2pos = 0.5;

    private boolean lbTriggered = false, rbTriggered = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = hardwareMap.get(Servo.class, "yaw1");
        s2 = hardwareMap.get(Servo.class, "yaw2");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            s1.setPosition(s1pos);
            s2.setPosition(s2pos);

            telemetry.addData("s1 position", s1pos);
            telemetry.addData("s2 position", s2pos);

            if (gamepad1.left_bumper && !lbTriggered) {
                s1pos+=0.05;
                s2pos-=0.05;
            }

            if (gamepad1.right_bumper && !rbTriggered) {
                s1pos-=0.05;
                s2pos+=0.05;
            }

            lbTriggered = gamepad1.left_bumper;
            rbTriggered = gamepad1.right_bumper;

            telemetry.update();
        }
    }
}