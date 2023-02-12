package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

public class RetrieveCone extends CommandBase {
    private final Intake intakeSubsystem;

    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private enum RetrieveState {
        GRABBING,
        RETRACTING_V4B,
        RETRACTING,
        IDLE
    }

    private RetrieveState retrieveState = RetrieveState.IDLE;

    public RetrieveCone(Intake subsystem) {
        intakeSubsystem = subsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.grab();
        eTime.reset();
        retrieveState = RetrieveState.GRABBING;
    }

    @Override
    public void execute() {
        switch (retrieveState) {
            case GRABBING:
                if (eTime.time() > 0.5) {
                    intakeSubsystem.setV4bPos(Intake.v4bRetractedPos);
                    retrieveState = RetrieveState.RETRACTING_V4B;
                }
                break;
            case RETRACTING_V4B:
                if (!intakeSubsystem.isV4BBusy()) {
                    intakeSubsystem.retractFully();
                    retrieveState = RetrieveState.RETRACTING;
                }
                break;
            case RETRACTING:
                if (!intakeSubsystem.isBusy()) {
                    retrieveState = RetrieveState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return retrieveState == RetrieveState.IDLE;
    }
}
