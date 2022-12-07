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

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
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
                    gamepad1.right_trigger,
                    gamepad1.left_trigger,
                    gamepad1.right_bumper,
                    gamepad1.left_bumper,
                    gamepad1.y,
                    gamepad1.b,
                    gamepad1.a,
                    gamepad1.x,
                    gamepad2.right_stick_y,
                    gamepad2.right_stick_x,
                    gamepad1.back
            );
        }
    }
}
