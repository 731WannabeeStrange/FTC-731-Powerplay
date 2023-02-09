package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class ResetLiftAndIntake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final LiftSubsystem liftSubsystem;

    public ResetLiftAndIntake(IntakeSubsystem intake, LiftSubsystem lift) {
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
