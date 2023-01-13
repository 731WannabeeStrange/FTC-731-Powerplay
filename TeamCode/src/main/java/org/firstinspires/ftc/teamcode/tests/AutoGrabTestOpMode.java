package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

@TeleOp
@Config
public class AutoGrabTestOpMode extends LinearOpMode
{
    public static double closedPosition = 0.7;
    public static double openPosition = 0.45;
    public static double distance = 2;

    private Servo claw;
    private RevColorSensorV3 colorSensor;

    private boolean lbTriggered = false, rbTriggered = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        claw = hardwareMap.get(Servo.class, "s1");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            if (colorSensor.getDistance(DistanceUnit.CM) < distance) {
                close();
            }

            if (gamepad1.left_bumper) {
                open();
            }

            Dashboard.packet.put("Distance", colorSensor.getDistance(DistanceUnit.CM));
            Dashboard.packet.put("Red", colorSensor.getNormalizedColors().red);
            Dashboard.packet.put("Green", colorSensor.getNormalizedColors().green);
            Dashboard.packet.put("Blue", colorSensor.getNormalizedColors().blue);

            telemetry.addData("Distance", colorSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Red", colorSensor.getNormalizedColors().red);
            telemetry.addData("Green", colorSensor.getNormalizedColors().green);
            telemetry.addData("Blue", colorSensor.getNormalizedColors().blue);

            Dashboard.periodic();

            telemetry.update();
        }
    }

    public void close() {
        claw.setPosition(closedPosition);
    }

    public void open() {
        claw.setPosition(openPosition);
    }
}