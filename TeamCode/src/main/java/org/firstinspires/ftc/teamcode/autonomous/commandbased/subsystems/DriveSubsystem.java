package org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class DriveSubsystem extends SubsystemBase {
    private final SampleMecanumDrive drive;

    public DriveSubsystem(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
    }

    public void startIMUThread(LinearOpMode opMode) {
        drive.startIMUThread(opMode);
    }

    public void periodic() {
        drive.update();
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d pose) {
        return drive.trajectorySequenceBuilder(pose);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }
}
