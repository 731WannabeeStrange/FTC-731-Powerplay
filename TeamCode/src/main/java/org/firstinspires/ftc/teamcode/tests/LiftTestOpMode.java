package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

@TeleOp
public class LiftTestOpMode extends LinearOpMode {
    Lift lift;
    MultipleTelemetry multipleTelemetry;

    private enum ManualLiftState {
        HIGH,
        MIDDLE,
        LOW,
        RETRACT
    }

    private ManualLiftState manualLiftState = ManualLiftState.RETRACT;

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new Lift(hardwareMap, multipleTelemetry);

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.y) {
                lift.extendHigh();
            } else if (gamepad1.b) {
                lift.extendMid();
            } else if (gamepad1.a) {
                lift.extendLow();
            } else if (gamepad1.x) {
                lift.retract();
            }

            lift.update();

            multipleTelemetry.addData("current lift state", manualLiftState);
            multipleTelemetry.addData("current lift position", lift.getSlidePosition());

            double[] motorPowers = lift.getMotorPowers();
            multipleTelemetry.addData("lift1 power", motorPowers[0]);
            multipleTelemetry.addData("lift2 power", motorPowers[1]);

            multipleTelemetry.update();
        }
    }
}
