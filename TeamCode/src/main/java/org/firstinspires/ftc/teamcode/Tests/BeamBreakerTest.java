package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Beam Breaker Test", group = "Sensor")
public class BeamBreakerTest extends LinearOpMode {

    DigitalChannel beamBreaker;

    @Override
    public void runOpMode() {
        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");

        beamBreaker.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {
            if (beamBreaker.getState()) {
                telemetry.addData("Beam Breaker", "Beam Broken");
            } else {
                telemetry.addData("Digital Touch", "Beam Not Broken");
            }

            telemetry.update();
        }
    }
}
