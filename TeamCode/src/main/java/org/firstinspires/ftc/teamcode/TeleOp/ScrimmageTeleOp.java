package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Lift;

public class ScrimmageTeleOp extends LinearOpMode {
    Drivetrain dt;
    Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            dt.driveRobot(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.left_bumper,
                    gamepad1.right_bumper,
                    gamepad1.dpad_up,
                    gamepad1.dpad_left,
                    gamepad1.dpad_down,
                    gamepad1.dpad_right
            );

            lift.lift(
                    gamepad1.y,
                    gamepad1.b,
                    gamepad1.a,
                    gamepad1.x,
                    gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0
            );
        }
    }
}
