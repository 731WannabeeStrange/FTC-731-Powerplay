package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Vision.AprilTagVisionPipeline;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Location;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Drivetrain;

@TeleOp
public class ScrimmageAuto extends LinearOpMode {
    AprilTagVisionPipeline pipeline;
    Location location = Location.LEFT;
    Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
    ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new AprilTagVisionPipeline();
        pipeline.init(hardwareMap, telemetry);
        while (!opModeIsActive() && !isStopRequested()) {
            location = pipeline.visionLoop(telemetry);
        }
        eTime.reset();
        switch (location) {
            case LEFT:
                while (eTime.time() < 1) {
                    drivetrain.driveRobotSimple(1, 0, 0);
                }
                eTime.reset();
                while (eTime.time() < 1) {
                    drivetrain.driveRobotSimple(0, -1, 0);
                }
                break;
            case MIDDLE:
                while (eTime.time() < 1) {
                    drivetrain.driveRobotSimple(1, 0, 0);
                }
                break;
            case RIGHT:
                while (eTime.time() < 1) {
                    drivetrain.driveRobotSimple(1, 0, 0);
                }
                eTime.reset();
                while (eTime.time() < 1) {
                    drivetrain.driveRobotSimple(0, 1, 0);
                }
                break;
        }
    }
}