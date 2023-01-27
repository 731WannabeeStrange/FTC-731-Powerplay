package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(group="test")
@Config
public class YawArmTest extends LinearOpMode
{
    private ServoImplEx s1;
    private ServoImplEx s2;

    private double pos = 0;

    private boolean lbTriggered = false, rbTriggered = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = hardwareMap.get(ServoImplEx.class, "s1");
        s2 = hardwareMap.get(ServoImplEx.class, "s2");

        s1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        s2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            double yawArmY = gamepad1.left_stick_y;
            double yawArmX = gamepad1.left_stick_x;
            if (yawArmY != 0 || yawArmX != 0) {
                double angle = Math.toDegrees(Math.atan2(yawArmY, yawArmX));

                if (angle < -180) {
                    angle += 360;
                }
                if (angle > 180) {
                    angle -= 360;
                }
                pos = (0.0037037 * angle) + 0.33333;


                s1.setPosition(pos);
                s2.setPosition(1 - pos);

                telemetry.addData("Servo 1 pos", pos);
                telemetry.addData("Servo 2 pos", 1 - pos);
                telemetry.addData("Angle", angle);
                telemetry.update();
            }
        }
    }
}