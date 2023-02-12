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
public class FixedRightAuto extends LinearOpMode {
    public static int numCycles = 3;

    enum AutoState {
        DRIVE_TO_SPOT,
        DROPPING,
        OPENING,
        GRAB_CONE,
        RETRIEVE_CONE,
        RETRIEVING,
        COLLECT,
        RELEASE_CONE,
        RELEASING,
        CLOSING,
        RETRACTING,
        WAIT_FOR_DEPOSIT,
        CHOOSE_PARK_LOCATION,
        PARK,
        IDLE;
    }
    private AutoState autoState = AutoState.DRIVE_TO_SPOT;

    private Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(90));

    private int cycle = 1;

    private SampleMecanumDrive drive;
    private Lift lift;
    private Intake intake;
    private AprilTagVisionPipeline pipeline;

    private Location location = Location.LEFT;
    private boolean parking = false;

    private ElapsedTime eTime = new ElapsedTime(MILLISECONDS);

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
            switch (autoState) {
                case DRIVE_TO_SPOT:
                    if (!drive.isBusy()) {
                        lift.setLiftState(Lift.LiftState.HIGH);
                        intake.extendAuto();
                        intake.setV4bPos(Intake.stackPositions[cycle - 1]);
                        intake.release();
                        autoState = AutoState.DROPPING;
                    }
                    break;

                case DROPPING:
                    if (lift.canControlArm()) {
                        lift.setYawArmAngle(-90);
                    }

                    if (!lift.isBusy() && !lift.isYawArmBusy()) {
                        lift.openGrabber();
                        eTime.reset();
                        autoState = AutoState.OPENING;
                    }
                    break;

                case OPENING:
                    if (!intake.isBusy() && eTime.time() > 500) {
                        lift.setLiftState(Lift.LiftState.RETRACT);

                        intake.grab();
                        telemetry.addLine("Closing claw");

                        autoState = AutoState.GRAB_CONE;
                    }
                    break;

                case GRAB_CONE:
                    if (!intake.isClawBusy()) {
                        telemetry.addLine("Retracting v4b");
                        intake.setV4bPos(Intake.v4bRetractedPos);
                        autoState = AutoState.RETRIEVE_CONE;
                    }
                    break;

                case RETRIEVE_CONE:
                    if (!intake.isV4BBusy()) {
                        telemetry.addLine("Retracting intake");
                        intake.retractPart(Intake.v4bRetractedPos);
                        autoState = AutoState.RETRIEVING;
                    }
                    break;

                case RETRIEVING:
                    if (!intake.isBusy()) {
                        lift.setLiftState(Lift.LiftState.COLLECT);
                        autoState = AutoState.COLLECT;
                    }
                    break;

                case COLLECT:
                    if (!lift.isBusy()) {
                        lift.closeGrabber();
                        eTime.reset();
                        autoState = AutoState.RELEASE_CONE;
                    }
                    break;

                case RELEASE_CONE:
                    if (eTime.time() > 500) {
                        intake.release();
                        autoState = AutoState.RELEASING;
                    }
                    break;

                case RELEASING:
                    if (!intake.isClawBusy()) {
                        eTime.reset();
                        autoState = AutoState.CLOSING;
                    }
                    break;

                case CLOSING:
                    if (eTime.time() > 0.5) {
                        intake.setV4bPos(Intake.v4bCompletelyRetractedPos);
                        autoState = AutoState.RETRACTING;
                    }
                    break;

                case RETRACTING:
                    if (!intake.isClawBusy() && !intake.isV4BBusy()) {
                        if (cycle < numCycles) {
                            eTime.reset();
                            cycle++;
                            autoState = AutoState.WAIT_FOR_DEPOSIT;
                        } else {
                            autoState = AutoState.CHOOSE_PARK_LOCATION;
                        }
                    }
                    break;

                case WAIT_FOR_DEPOSIT:
                    if (eTime.time() > 200) {
                        lift.setLiftState(Lift.LiftState.HIGH);
                        intake.extendAuto();
                        intake.setV4bPos(Intake.stackPositions[cycle - 1]);
                        intake.release();
                        autoState = AutoState.DROPPING;
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
                    lift.setLiftState(Lift.LiftState.ZERO);
                    intake.retractFully();
                    autoState = AutoState.PARK;
                    parking = true;
                    break;

                case PARK:
                    if (!drive.isBusy()) {
                        autoState = AutoState.IDLE;
                    }
                    break;

                case IDLE:
                    break;
            }

            if (getRuntime() - startTime > 25 && !parking) {
                autoState = AutoState.CHOOSE_PARK_LOCATION;
            }

            drive.update();
            lift.periodic();
            intake.periodic();

            telemetry.addData("State", autoState);
            telemetry.addData("Intake ticks", intake.getSlidePosition());
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();

        }
    }
}
