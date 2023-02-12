package org.firstinspires.ftc.teamcode.autonomous;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.signal.AprilTagVisionPipeline;
import org.firstinspires.ftc.teamcode.vision.signal.Location;

@Autonomous(preselectTeleOp = "CompTeleOp")
public class RightAuto extends LinearOpMode {
    public static int numCycles = 3;


    enum State {
        DRIVE_TO_SPOT,
        DEPOSIT,
        DEPOSIT_2,
        OPENING,
        GRAB_CONE,
        COLLECT,
        RELEASE_CONE,
        CLOSING,
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
    boolean flag2 = false;
    boolean parking = false;

    ElapsedTime eTime = new ElapsedTime(MILLISECONDS);
    ElapsedTime intakeTimer = new ElapsedTime(MILLISECONDS);
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

        intake.setV4bPos(Intake.v4bCompletelyRetractedPos);

        TrajectorySequence driveToSpot = drive.trajectorySequenceBuilder(startPose)
                .back(36)
                .splineToSplineHeading(new Pose2d(-30, 12.5, Math.toRadians(180)), Math.toRadians(0))
                .back(4)
                .waitSeconds(1)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(driveToSpot.end())
                .lineTo(new Vector2d(-12, 12))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence midPark = drive.trajectorySequenceBuilder(driveToSpot.end())
                .lineTo(new Vector2d(-36, 12))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(driveToSpot.end())
                .lineTo(new Vector2d(-60, 12))
                .turn(Math.toRadians(90))
                .build();

        while (opModeInInit()) {
            location = pipeline.visionLoop(telemetry);
        }

        drive.followTrajectorySequenceAsync(driveToSpot);
        lift.setLiftState(Lift.LiftState.RETRACT);

        double startTime = getRuntime();

        while (opModeIsActive()) {
            switch (state) {
                case DRIVE_TO_SPOT:
                    if (!drive.isBusy()) {
                        state = State.DEPOSIT;
                    }
                    break;

                case DEPOSIT:
                    deposit();
                    state = State.WAIT;
                    nextState = State.DEPOSIT_2;
                    waitTime = 2000;
                    flag2 = true;
                    eTime.reset();
                    break;

                case DEPOSIT_2:
                    deposit();
                    if (!lift.isBusy() && !intake.isBusy()) {
                        lift.openGrabber();
                        lift.periodic();
                        eTime.reset();
                        state = State.OPENING;
                    }
                    break;

                case OPENING:
                    if (eTime.time() > 0.5) {
                        state = State.WAIT;
                        nextState = State.GRAB_CONE;
                        waitTime = 200;
                        eTime.reset();
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
                    lift.periodic();

                    if (!lift.isBusy()) {
                        lift.closeGrabber();
                        eTime.reset();
                        state = State.RELEASE_CONE;
                    }
                    break;

                case RELEASE_CONE:
                    if (eTime.time() > 500) {
                        intake.release();
                        if (!intake.isClawBusy()) {
                            eTime.reset();
                            state = State.CLOSING;
                        }
                    }
                    break;

                case CLOSING:
                    if (eTime.time() > 0.5) {
                        intake.setV4bPos(Intake.v4bCompletelyRetractedPos);
                        intake.periodic();
                        if (!intake.isClawBusy() && !intake.isV4BBusy()) {
                            if (cycle < numCycles) {
                                state = State.WAIT;
                                nextState = State.DEPOSIT;
                                waitTime = 200;
                                eTime.reset();
                                intakeTimer.reset();
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
                        lift.setLiftState(Lift.LiftState.ZERO);
                        intake.retractFully();
                        state = State.IDLE;
                    }
                    break;

                case WAIT:
                    if (eTime.time() > waitTime) {
                        state = nextState;
                        flag2 = false;
                    }
                    if (flag2) {
                        deposit();
                    }

                    break;

                case IDLE:
                    break;
            }

            if (getRuntime() - startTime > 25 && !parking) {
                state = State.CHOOSE_PARK_LOCATION;
                intake.retractPart(Intake.v4bCompletelyRetractedPos);
                lift.setLiftState(Lift.LiftState.RETRACT);
            }

            drive.update();
            lift.periodic();
            intake.periodic();

            telemetry.addData("State", state);
            telemetry.addData("Intake ticks", intake.getSlidePosition());
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();

        }
    }

    public void deposit() {
        if (intakeTimer.time() > 500) {
            intake.extendFully();
            intake.setV4bPos(Intake.stackPositions[cycle - 1]);
            intake.release();
        }
        if (intake.isConeDetected()) {
            intake.stopSlides();
        }
        lift.setLiftState(Lift.LiftState.HIGH);
        if (lift.getSlidePosition() > Lift.liftMid) {
            lift.setYawArmAngle(-90);
        }

        intake.periodic();
        lift.periodic();
    }

    public void grabCone() {
        lift.setYawArmAngle(Lift.yawArmAngle);
        lift.periodic();
        if (!lift.isYawArmBusy()) {
            lift.setLiftState(Lift.LiftState.RETRACT);
            lift.periodic();
        }
        intake.grab();
        intake.periodic();
        telemetry.addLine("Closing claw");
        if (!intake.isClawBusy()) {
            telemetry.addLine("Setting v4b pos");
            intake.setV4bPos(Intake.v4bRetractedPos);
            intake.periodic();
            if (!intake.isV4BBusy()) {
                telemetry.addLine("Retracting intake");
                intake.retractPart(Intake.v4bRetractedPos);
                intake.periodic();
                if (!intake.isBusy() && !lift.isBusy()) {
                    flag = true;
                }
            }
        }
    }
}
