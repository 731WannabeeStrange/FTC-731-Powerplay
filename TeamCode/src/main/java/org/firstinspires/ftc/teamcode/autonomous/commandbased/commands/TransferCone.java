package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class TransferCone extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final LiftSubsystem liftSubsystem;

    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private enum TransferState {
        DROPPING,
        GRABBING,
        RELEASING,
        RETRACTING,
        IDLE
    }

    private TransferState transferState = TransferState.IDLE;

    public TransferCone(IntakeSubsystem intake, LiftSubsystem lift) {
        intakeSubsystem = intake;
        liftSubsystem = lift;
        addRequirements(intakeSubsystem, liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.setLiftState(Lift.LiftState.COLLECT);
        transferState = TransferState.DROPPING;
    }

    @Override
    public void execute() {
        switch (transferState) {
            case DROPPING:
                if (!liftSubsystem.isBusy()) {
                    liftSubsystem.closeGrabber();
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
                    intakeSubsystem.setV4bPos(IntakeSubsystem.v4bCompletelyRetractedPos);
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
