package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.trajectorysequence.TrajectorySequence;

public class FollowTrajectory extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final TrajectorySequence sequence;

    public FollowTrajectory(DriveSubsystem subsystem, TrajectorySequence trajectorySequence) {
        driveSubsystem = subsystem;
        sequence = trajectorySequence;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.followTrajectorySequenceAsync(sequence);
    }

    @Override
    public boolean isFinished() {
        return !driveSubsystem.isBusy();
    }
}
