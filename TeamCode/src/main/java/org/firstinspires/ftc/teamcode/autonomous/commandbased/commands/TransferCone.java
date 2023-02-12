package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class TransferCone extends CommandBase {
    private final Intake intakeSubsystem;
    private final Lift liftSubsystem;

    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private enum TransferState {
        RELEASE_CONE,
        DROPPING,
        GRABBING,
        RELEASING,
        RETRACTING,
        IDLE
    }

    private TransferState transferState = TransferState.IDLE;

    public TransferCone(Intake intake, Lift lift) {
        intakeSubsystem = intake;
        liftSubsystem = lift;
        addRequirements(intakeSubsystem, liftSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.release();
        transferState = TransferState.RELEASE_CONE;
    }

    @Override
    public void execute() {
        switch (transferState) {
            case RELEASE_CONE:
                if (!intakeSubsystem.isClawBusy()) {
                    intakeSubsystem.setV4bPos(Intake.v4bCompletelyRetractedPos);
                    liftSubsystem.closeGrabber();
                    liftSubsystem.setLiftState(Lift.LiftState.COLLECT);
                    transferState = TransferState.DROPPING;
                }
                break;
            case DROPPING:
                if (!liftSubsystem.isBusy()) {
                    eTime.reset();
                    transferState = TransferState.GRABBING;
                }
                break;
            case GRABBING:
                if (eTime.time() > 0.5) {
                    intakeSubsystem.release();
                    transferState = TransferState.RELEASING;
                }
                break;
            case RELEASING:
                if (!intakeSubsystem.isClawBusy()) {
                    intakeSubsystem.setV4bPos(Intake.v4bCompletelyRetractedPos);
                    transferState = TransferState.RETRACTING;
                }
                break;
            case RETRACTING:
                if (!intakeSubsystem.isV4BBusy()) {
                    transferState = TransferState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return transferState == TransferState.IDLE;
    }
}
