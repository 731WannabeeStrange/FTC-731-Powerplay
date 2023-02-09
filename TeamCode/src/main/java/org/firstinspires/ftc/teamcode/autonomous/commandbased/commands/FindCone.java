package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.IntakeSubsystem;

public class FindCone extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final double v4bpos;

    private enum FindState {
        DROPPING,
        EXTENDING,
        IDLE
    }

    private FindState findState = FindState.IDLE;

    public FindCone(IntakeSubsystem subsystem, double pos) {
        intakeSubsystem = subsystem;
        v4bpos = pos;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setV4bPos(v4bpos);
        intakeSubsystem.release();
        findState = FindState.DROPPING;
    }

    @Override
    public void execute() {
        switch (findState) {
            case DROPPING:
                if (!intakeSubsystem.isClawBusy() && !intakeSubsystem.isV4BBusy()) {
                    intakeSubsystem.extendFully();
                    findState = FindState.EXTENDING;
                }
            case EXTENDING:
                if (intakeSubsystem.isConeDetected() || intakeSubsystem.getSlidePosition() > IntakeSubsystem.maxExtension) {
                    intakeSubsystem.stopSlides();
                    findState = FindState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return findState == FindState.IDLE;
    }
}
