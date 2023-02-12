package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class ResetLiftAndIntake extends CommandBase {
    private final Intake intakeSubsystem;
    private final Lift liftSubsystem;

    public ResetLiftAndIntake(Intake intake, Lift lift) {
        intakeSubsystem = intake;
        liftSubsystem = lift;
        addRequirements(intakeSubsystem, liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.setLiftState(Lift.LiftState.ZERO);
        intakeSubsystem.retractFully();
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.isBusy() && !intakeSubsystem.isV4BBusy() && !liftSubsystem.isBusy();
    }
}
