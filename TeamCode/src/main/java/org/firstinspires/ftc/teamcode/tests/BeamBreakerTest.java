package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Beam Breaker Test", group = "Sensor")
public class BeamBreakerTest extends LinearOpMode {

    DigitalChannel beamBreaker;

    @Override
    public void runOpMode() {
        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();

        while (opModeIsActive()) {
            if (beamBreaker.getState()) {
                telemetry.addData("Beam Breaker getState()", "<p style=\"color:rgb(255,0,0);\">TRUE</p>");
            } else {
                telemetry.addData("Beam Breaker getState()", "<p style=\"color:rgb(255,0,0);\">FALSE</p>");
            }

            telemetry.update();
        }
    }
}
