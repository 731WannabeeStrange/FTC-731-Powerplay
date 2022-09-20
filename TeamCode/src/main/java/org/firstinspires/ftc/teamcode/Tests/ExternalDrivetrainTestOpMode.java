package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Drivetrain;

public class ExternalDrivetrainTestOpMode extends LinearOpMode {
    private Drivetrain drivetrain;

    private enum driveMode {
        DRIVER_CONTROLLED,
        AUTO_CONTROL
    }

    private driveMode driveState = driveMode.AUTO_CONTROL;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        int desiredAngle = 0;

        waitForStart();

        while (opModeIsActive()) {
            switch (driveState) {
                case DRIVER_CONTROLLED:
                    drivetrain.driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper, false, desiredAngle);
                    if (!gamepad1.right_bumper) {
                        if (gamepad1.dpad_up) {
                            desiredAngle = 0;
                            driveState = driveMode.AUTO_CONTROL;
                        }
                        if (gamepad1.dpad_right) {
                            desiredAngle = 270;
                            driveState = driveMode.AUTO_CONTROL;
                        }
                        if (gamepad1.dpad_down) {
                            desiredAngle = 180;
                            driveState = driveMode.AUTO_CONTROL;
                        }
                        if (gamepad1.dpad_left) {
                            desiredAngle = 90;
                            driveState = driveMode.AUTO_CONTROL;
                        }
                    }
                    break;
                case AUTO_CONTROL:
                    if (gamepad1.right_stick_x != 0) {
                        driveState = driveMode.DRIVER_CONTROLLED;
                        break;
                    }
                    if (!gamepad1.right_bumper) {
                        if (gamepad1.dpad_up) {
                            desiredAngle = 0;
                        }
                        if (gamepad1.dpad_right) {
                            desiredAngle = 270;
                        }
                        if (gamepad1.dpad_down) {
                            desiredAngle = 180;
                        }
                        if (gamepad1.dpad_left) {
                            desiredAngle = 90;
                        }
                    }
                    drivetrain.driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper, true, desiredAngle);
                    break;
            }

            telemetry.update();
        }
    }
}
