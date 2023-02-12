package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

@TeleOp(group="test")
@Config
public class LinkageTest extends LinearOpMode
{
    private Servo s1;

    private double s1pos = 0.5;

    private boolean lbTriggered = false, rbTriggered = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = hardwareMap.get(Servo.class, "extension");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            s1.setPosition(s1pos);

            telemetry.addData("s1 position", s1pos);

            if (gamepad1.left_bumper && !lbTriggered) {
                s1pos -= 0.05;
            }

            if (gamepad1.right_bumper && !rbTriggered) {
                s1pos += 0.05;
            }

            lbTriggered = gamepad1.left_bumper;
            rbTriggered = gamepad1.right_bumper;

            telemetry.update();
        }
    }
}