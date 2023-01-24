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
public class PreloadParkAuto extends LinearOpMode {
    public static int numCycles = 6;


    enum State {
        DRIVE_TO_SPOT,
        DEPOSIT,
        PARK,
        WAIT,
        IDLE
    }
    public State state = State.DRIVE_TO_SPOT;

    Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(90));

    private int cycle = 1;

    SampleMecanumDrive drive;
    Lift lift;
    AprilTagVisionPipeline pipeline;

    Location location = Location.LEFT;

    boolean flag = false;

    ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double waitTime = 0;
    State nextState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        pipeline = new AprilTagVisionPipeline();

        pipeline.init(hardwareMap, telemetry);

        drive.setPoseEstimate(startPose);

        TrajectorySequence driveToSpot = drive.trajectorySequenceBuilder(startPose)
                .back(36)
                .splineToSplineHeading(new Pose2d(-30, 12, Math.toRadians(180)), Math.toRadians(0))
                .back(6)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(driveToSpot.end())
                .back(12)
                .build();

        TrajectorySequence midPark = drive.trajectorySequenceBuilder(driveToSpot.end())
                .forward(12)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(driveToSpot.end())
                .forward(36)
                .build();

        while (opModeInInit()) {
            location = pipeline.visionLoop(telemetry);
        }

        drive.followTrajectorySequenceAsync(driveToSpot);

        while (opModeIsActive()) {
            switch (state) {
                case DRIVE_TO_SPOT:
                    if (!drive.isBusy()) {
                        state = State.DEPOSIT;
                    }
                    break;

                case DEPOSIT:
                    deposit();
                    telemetry.addData("lift busy", lift.isBusy());
                    if (!lift.isBusy()) {
                        lift.deposit();
                        if (!lift.grabber.isBusy()) {
                            state = State.WAIT;
                            nextState = State.PARK;
                            waitTime = 500;
                        }
                    }
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        state = State.IDLE;
                    }
                    break;

                case WAIT:
                    if (eTime.time() > waitTime) {
                        state = nextState;
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

                case IDLE:
                    break;
            }

            drive.update();
            lift.update();

            telemetry.addData("State", state);
            telemetry.update();

        }
    }

    public void deposit() {
        lift.extendHigh();
        if (lift.getSlidePosition() > Lift.minHeightForArmRotation) {
            lift.setYawArmAngle(-90);
        }
    }
}
