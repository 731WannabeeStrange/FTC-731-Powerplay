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
public class RightAuto extends LinearOpMode {
    public static int numCycles = 6;


    enum State {
        DRIVE_TO_SPOT,
        DEPOSIT,
        GRAB_CONE,
        COLLECT,
        CHOOSE_PARK_LOCATION,
        PARK,
        WAIT,
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

    boolean flag = false;
    boolean parking = false;

    ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double waitTime = 0;
    State nextState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
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
        intake.setV4bPos(Intake.v4bRetractedPos - 0.1);

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
                    telemetry.addData("intake busy", intake.isBusy());
                    if (!lift.isBusy() && !intake.isBusy()) {
                        lift.deposit();
                        lift.update();
                        if (!lift.isGrabberBusy()) {
                            state = State.WAIT;
                            nextState = State.GRAB_CONE;
                            waitTime = 200;
                            eTime.reset();
                        }
                    }
                    break;

                case GRAB_CONE:
                    grabCone();

                    if (flag) {
                        state = State.WAIT;
                        nextState = State.COLLECT;
                        waitTime = 200;
                        eTime.reset();
                        flag = false;
                    }

                    break;

                case COLLECT:
                    lift.setLiftState(Lift.LiftState.COLLECT);
                    lift.update();

                    if (!lift.isBusy()) {
                        intake.release();
                        intake.update();

                        if (!intake.isClawBusy()) {
                            if (cycle < numCycles) {
                                state = State.WAIT;
                                nextState = State.DEPOSIT;
                                waitTime = 200;
                                eTime.reset();
                                cycle++;
                            } else {
                                state = State.CHOOSE_PARK_LOCATION;

                            }
                        }

                    }
                    break;

                case CHOOSE_PARK_LOCATION:
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
                    state = State.PARK;
                    parking = true;
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        state = State.IDLE;
                    }
                    break;

                case WAIT:
                    if (eTime.time() > waitTime) {
                        state = nextState;
                    }
                    break;

                case IDLE:
                    break;
            }

            if (getRuntime() > 27 && !parking) {
                state = State.CHOOSE_PARK_LOCATION;
                intake.retractPart(Intake.v4bRetractedPos);
                lift.setLiftState(Lift.LiftState.RETRACT);
            }

            drive.update();
            lift.update();
            intake.update();

            telemetry.addData("State", state);
            telemetry.addData("Intake ticks", intake.getSlidePosition());
            telemetry.update();

        }
    }

    public void deposit() {
        intake.extendFully();
        intake.setV4bPos(Intake.stackPositions[cycle - 1]);
        intake.release();
        lift.setLiftState(Lift.LiftState.HIGH);
        if (lift.getSlidePosition() > Lift.liftMid) {
            lift.setYawArmAngle(-90);
        }

        intake.update();
        lift.update();
    }

    public void grabCone() {
        lift.setYawArmAngle(-10);
        lift.update();
        if (!lift.isYawArmBusy()) {
            lift.setLiftState(Lift.LiftState.RETRACT);
            lift.update();
        }
        intake.grab();
        intake.update();
        telemetry.addLine("Closing claw");
        if (!intake.isClawBusy()) {
            telemetry.addLine("Setting v4b pos");
            intake.setV4bPos(Intake.v4bRetractedPos);
            intake.update();
            if (!intake.isV4BBusy()) {
                telemetry.addLine("Retracting intake");
                intake.retractPart(Intake.v4bRetractedPos);
                intake.update();
                if (!intake.isBusy() && !lift.isBusy()) {
                    flag = true;
                }
            }
        }
    }
}
