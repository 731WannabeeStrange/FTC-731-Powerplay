package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Rumbler;
import org.firstinspires.ftc.teamcode.robot.subsystems.ScoringMech;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

@TeleOp(group="a")
public class CompTeleOp extends LinearOpMode {
    Drivetrain dt;
    ScoringMech sm;
    Rumbler rb;

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rb = new Rumbler(gamepad1);
        dt = new Drivetrain(hardwareMap, multipleTelemetry);
        sm = new ScoringMech(hardwareMap, rb, multipleTelemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (sm.isControllingArm()) {
                dt.driveRobot(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        gamepad1.left_bumper,
                        gamepad2.dpad_up,
                        gamepad2.dpad_left,
                        gamepad2.dpad_down,
                        gamepad2.dpad_right
                );
                sm.score(
                        gamepad1.left_trigger > 0,
                        gamepad1.y,
                        gamepad1.b,
                        gamepad1.a,
                        gamepad1.right_bumper,
                        gamepad2.right_stick_y,
                        gamepad2.right_stick_x,
                        gamepad1.back,
                        gamepad1.dpad_down,
                        gamepad1.dpad_right,
                        gamepad1.dpad_up,
                        gamepad1.dpad_left
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
                sm.score(
                        gamepad1.left_trigger > 0,
                        gamepad1.y,
                        gamepad1.b,
                        gamepad1.a,
                        gamepad1.right_bumper,
                        gamepad2.right_stick_y,
                        gamepad2.right_stick_x,
                        gamepad1.back,
                        gamepad2.dpad_down,
                        gamepad2.dpad_right,
                        gamepad2.dpad_up,
                        gamepad2.dpad_left
                );
            }

        }

        Dashboard.periodic();
    }
}
