package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.signal.AprilTagVisionPipeline;
import org.firstinspires.ftc.teamcode.vision.signal.Location;

@Autonomous
public class ParkAuto extends LinearOpMode {

    enum State {
        DRIVE_TO_SPOT,
        DEPOSIT,
        PARK,
        WAIT,
        IDLE
    }
    public State state = State.DRIVE_TO_SPOT;

    Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(90));


    SampleMecanumDrive drive;
    AprilTagVisionPipeline pipeline;

    Location location = Location.LEFT;

    ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double waitTime = 0;
    State nextState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        pipeline = new AprilTagVisionPipeline();

        pipeline.init(hardwareMap, telemetry);

        drive.setPoseEstimate(startPose);

        TrajectorySequence driveToSpot = drive.trajectorySequenceBuilder(startPose)
                .back(28)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(driveToSpot.end())
                .turn(Math.toRadians(90))
                .back(24)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence midPark = drive.trajectorySequenceBuilder(driveToSpot.end())
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(driveToSpot.end())
                .turn(Math.toRadians(-90))
                .back(24)
                .turn(Math.toRadians(-90))
                .build();

        while (opModeInInit()) {
            location = pipeline.visionLoop(telemetry);
        }

        drive.followTrajectorySequenceAsync(driveToSpot);

        while (opModeIsActive()) {
            switch (state) {
                case DRIVE_TO_SPOT:
                    if (!drive.isBusy()) {
                        state = State.PARK;
                        switch (location) {
                            case LEFT:
                                drive.followTrajectorySequenceAsync(leftPark);
                                break;
                            case MIDDLE:
                                drive.followTrajectorySequenceAsync(midPark);
                                break;
                            case RIGHT:
                                drive.followTrajectorySequenceAsync(rightPark);
                                break;
                        }
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        state = State.IDLE;
                    }
                    break;

                case IDLE:
                    break;
            }

            drive.update();

            telemetry.addData("State", state);
            telemetry.update();

        }
    }
}
