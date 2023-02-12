package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

public class FindCone extends CommandBase {
    private final Intake intakeSubsystem;
    private final double v4bpos;

    private enum FindState {
        WAITING,
        DROPPING,
        EXTENDING,
        IDLE
    }

    private FindState findState = FindState.IDLE;

    private ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public FindCone(Intake subsystem, double pos) {
        intakeSubsystem = subsystem;
        v4bpos = pos;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        eTime.reset();
        findState = FindState.WAITING;
    }

    @Override
    public void execute() {
        switch (findState) {
            case WAITING:
                if (eTime.time() > 1) {
                    intakeSubsystem.setV4bPos(v4bpos);
                    intakeSubsystem.release();
                    findState = FindState.DROPPING;
                }
                break;
            case DROPPING:
                if (!intakeSubsystem.isClawBusy() && !intakeSubsystem.isV4BBusy()) {
                    intakeSubsystem.extendFully();
                    findState = FindState.EXTENDING;
                }
                break;
            case EXTENDING:
                if (intakeSubsystem.isConeDetected() || intakeSubsystem.getSlidePosition() > Intake.maxExtension) {
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
