package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

@TeleOp(group="test")
public class IntakeTunerOpMode extends LinearOpMode {
    MultipleTelemetry multipleTelemetry;
    Intake intake;
    Lift lift; //only here to make sure it is out of the way when moving intake

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap, multipleTelemetry);
        lift = new Lift(hardwareMap, multipleTelemetry);
        
        while (opModeIsActive() && !isStopRequested()) {
            if (!lift.isBusy()) {
                if (gamepad1.b) {
                    intake.extendFully();
                } else if (gamepad1.x) {
                    intake.retractFully();
                }
            }

            lift.update();
            intake.update();

            multipleTelemetry.addData("current intake position", intake.getSlidePosition());

            double[] motorPowers = intake.getMotorPowers();
            multipleTelemetry.addData("intake1 power", motorPowers[0]);
            multipleTelemetry.addData("intake2 power", motorPowers[1]);

            multipleTelemetry.update();
        }
    }
}
