package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMech;

@TeleOp
public class ScrimmageTeleOp extends LinearOpMode {
    Drivetrain dt;
    ScoringMech scoring;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dt = new Drivetrain(hardwareMap, telemetry);
        scoring = new ScoringMech(hardwareMap, multipleTelemetry);

        boolean previous2a = false;
        boolean oneGamepad = true;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad2.a && !previous2a) {
                oneGamepad = !oneGamepad;
            }

            if (oneGamepad) {
                dt.driveRobot(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        0,
                        gamepad1.left_bumper,
                        gamepad1.dpad_up,
                        gamepad1.dpad_left,
                        gamepad1.dpad_down,
                        gamepad1.dpad_right
                );

                scoring.score(
                        gamepad1.left_trigger > 0,
                        gamepad1.left_bumper,
                        gamepad1.right_trigger > 0,
                        gamepad1.y,
                        gamepad1.b,
                        gamepad1.a,
                        gamepad1.right_bumper,
                        gamepad1.right_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.back,
                        false,
                        false,
                        false,
                        false
                );
            } else {
                dt.driveRobot(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        gamepad1.left_bumper,
                        gamepad1.dpad_up,
                        gamepad1.dpad_left,
                        gamepad1.dpad_down,
                        gamepad1.dpad_right
                );

                scoring.score(
                        gamepad1.left_trigger > 0,
                        gamepad1.left_bumper,
                        gamepad1.right_trigger > 0,
                        gamepad1.y,
                        gamepad1.b,
                        gamepad1.a,
                        gamepad1.right_bumper,
                        gamepad1.right_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.back,
                        gamepad2.dpad_down,
                        gamepad2.dpad_right,
                        gamepad2.dpad_up,
                        gamepad2.dpad_left
                );
            }

            previous2a = gamepad2.a;
        }
    }
}
