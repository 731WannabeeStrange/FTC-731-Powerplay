package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

@TeleOp
public class ScrimmageTeleOpNoEncoders extends LinearOpMode {
    Drivetrain dt;
    Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dt = new Drivetrain(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);

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

            lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            if (gamepad2.right_stick_y != 0 || gamepad2.right_stick_x != 0) {
                lift.setYawArmAngle(Math.atan2(gamepad2.right_stick_y, gamepad2.right_stick_x));
            }
            if (gamepad2.dpad_right) {
                lift.setYawArmAngle(0);
            } else if (gamepad2.dpad_up) {
                lift.setYawArmAngle(90);
            } else if (gamepad2.dpad_left) {
                lift.setYawArmAngle(180);
            } else if (gamepad2.dpad_down) {
                lift.setYawArmAngle(-90);
            }

            if (gamepad1.right_bumper) {
                lift.deposit();
            } else if (gamepad1.left_bumper) {
                lift.grab();
            }
        }
    }
}
