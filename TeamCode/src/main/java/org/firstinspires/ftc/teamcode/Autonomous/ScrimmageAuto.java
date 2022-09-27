package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Vision.AprilTagVisionPipeline;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Location;

@TeleOp
public class ScrimmageAuto extends LinearOpMode {
    AprilTagVisionPipeline pipeline;
    Location location = Location.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new AprilTagVisionPipeline();
        pipeline.init(hardwareMap, telemetry);
        while (!opModeIsActive() && !isStopRequested()) {
            location = pipeline.visionLoop(telemetry);
        }
        switch (location) {
            case LEFT:
                
                break;
            case MIDDLE:
                break;
            case RIGHT:
                break;
        }
    }
}