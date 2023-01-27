package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.signal.AprilTagVisionPipeline;
import org.firstinspires.ftc.teamcode.vision.signal.Location;

@TeleOp(group="test")
public class AprilTagTest extends LinearOpMode {
    AprilTagVisionPipeline pipeline;
    Location location = Location.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new AprilTagVisionPipeline();
        pipeline.init(hardwareMap, telemetry);
        while (!opModeIsActive() && !isStopRequested()) {
            location = pipeline.visionLoop(telemetry);
        }
    }
}