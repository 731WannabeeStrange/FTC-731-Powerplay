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
                manualLiftState = ManualLiftState.HIGH;
            } else if (gamepad1.b) {
                manualLiftState = ManualLiftState.MIDDLE;
            } else if (gamepad1.a) {
                manualLiftState = ManualLiftState.LOW;
            } else if (gamepad1.x) {
                manualLiftState = ManualLiftState.RETRACT;
            }

            switch (manualLiftState) {
                case HIGH:
                    lift.extendHigh();
                    break;
                case MIDDLE:
                    lift.extendMid();
                    break;
                case LOW:
                    lift.extendLow();
                    break;
                case RETRACT:
                    lift.retract();
                    break;
            }

            multipleTelemetry.addData("current lift state", manualLiftState);
            multipleTelemetry.addData("current lift position", lift.getSlidePosition());
            multipleTelemetry.update();
        }
    }
}
