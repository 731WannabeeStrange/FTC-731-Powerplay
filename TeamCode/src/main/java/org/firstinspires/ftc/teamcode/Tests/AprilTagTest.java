package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Vision.AprilTagVisionPipeline;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Location;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Drivetrain;

@TeleOp(name="AprilTagTest")
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