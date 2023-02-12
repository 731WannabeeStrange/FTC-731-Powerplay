package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class GoToLiftState extends CommandBase {
    private final Lift liftSubsystem;
    private final Lift.LiftState desiredState;

    public GoToLiftState(Lift subsystem, Lift.LiftState state) {
        liftSubsystem = subsystem;
        desiredState = state;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.setLiftState(desiredState);
    }

    // no execute() or isFinished() because this is used as a default command that should never end
    // unless interrupted
}
