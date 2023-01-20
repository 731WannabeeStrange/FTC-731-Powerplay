package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.signal.AprilTagVisionPipeline;
import org.firstinspires.ftc.teamcode.vision.signal.Location;

@Autonomous
public class RightAuto extends LinearOpMode {
    public static int numCycles = 2;


    enum State {
        DRIVE_TO_SPOT,
        DEPOSIT,
        GRAB_CONE,
        COLLECT,
        PARK,
        IDLE
    }
    public State state = State.DRIVE_TO_SPOT;

    Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(90));

    private int cycle = 1;

    SampleMecanumDrive drive;
    Lift lift;
    Intake intake;
    AprilTagVisionPipeline pipeline;

    Location location = Location.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        pipeline = new AprilTagVisionPipeline();

        pipeline.init(hardwareMap, telemetry);

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
                    if (!lift.isBusy() && !lift.yawArm.isBusy()) {
                        lift.deposit();
                        if (!lift.grabber.isBusy()) {
                            state = State.GRAB_CONE;
                        }
                    }
                    break;

                case GRAB_CONE:
                    grabCone();

                    if (!lift.isBusy() && !intake.isBusy() && !intake.v4b.isBusy()) {
                        state = State.COLLECT;
                    }

                    break;

                case COLLECT:
                    lift.collect();

                    if (!lift.isBusy()) {
                        intake.release();

                        if (cycle < numCycles) {
                            state = State.DEPOSIT;
                            cycle++;
                        } else {
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

            lift.update();

        }
    }

    public void deposit() {
        intake.extendTicks(Intake.maxExtension, 0.5);
        lift.extendHigh();
        if (lift.getSlidePosition() > Lift.minHeightForArmRotation) {
            lift.setYawArmAngle(-90);
        }
    }

    public void grabCone() {
        lift.retract();
        intake.grab();
        if (!intake.claw.isBusy()) {
            intake.retractFully();
        }
    }
}
