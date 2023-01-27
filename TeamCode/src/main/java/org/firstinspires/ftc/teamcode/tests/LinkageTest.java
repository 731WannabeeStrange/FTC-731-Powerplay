package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group="test")
@Config
public class LinkageTest extends LinearOpMode
{
    private Servo s1;
    private Servo s2;
    private Servo grabber;

    private double s1pos = 1;
    private double s2pos = 0;
    private double grabberPos = 0.4;

    private boolean lbTriggered = false, rbTriggered = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        grabber = hardwareMap.get(Servo.class, "grab");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            s1.setPosition(s1pos);
            s2.setPosition(s2pos);
            grabber.setPosition(grabberPos);

            telemetry.addData("s1 position", s1pos);
            telemetry.addData("s2 position", s2pos);
            telemetry.addData("grab position", grabberPos);

            if (gamepad1.left_bumper && !lbTriggered) {
                s1pos=1;
                s2pos=0;
            }

            if (gamepad1.right_bumper && !rbTriggered) {
                s1pos=0.3;
                s2pos=0.7;
            }

            if (gamepad1.a) {
                grabberPos = 0.25;
            }

            if (gamepad1.b) {
                grabberPos = 0.4;
            }

            lbTriggered = gamepad1.left_bumper;
            rbTriggered = gamepad1.right_bumper;

            telemetry.update();
        }
    }
}