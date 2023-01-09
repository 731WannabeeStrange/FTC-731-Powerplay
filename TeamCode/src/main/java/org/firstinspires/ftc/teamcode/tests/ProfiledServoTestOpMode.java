package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.ProfiledServo;
import org.firstinspires.ftc.teamcode.utils.MotionConstraint;

@TeleOp
@Config
public class ProfiledServoTestOpMode extends LinearOpMode
{
    private ProfiledServo servo;

    private double initialPos = 1;
    private double finalPos = 0;

    private double position = initialPos;

    private boolean lbTriggered = false, rbTriggered = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = new ProfiledServo(hardwareMap, "s1", new MotionConstraint(0.5, 0.5, 0.5), initialPos);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            servo.setPosition(position);

            telemetry.addData("servo position", position);
            telemetry.addData("is busy", servo.isBusy());
            telemetry.addData("constraints", servo.constraints);

            if (gamepad1.left_bumper && !lbTriggered) {
                position = initialPos;
            }

            if (gamepad1.right_bumper && !rbTriggered) {
                position = finalPos;
            }

            lbTriggered = gamepad1.left_bumper;
            rbTriggered = gamepad1.right_bumper;

            servo.periodic();

            telemetry.update();
        }
    }
}