package org.firstinspires.ftc.teamcode.autonomous.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.commandbased.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class DepositCone extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    private final Lift.LiftState desiredState;
    private final int yawArmAngle;

    private final ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private enum DepositState {
        RAISING,
        MOVING_ARM,
        DROPPING,
        RETRACTING,
        IDLE
    }

    private DepositState depositState = DepositState.IDLE;

    public DepositCone(LiftSubsystem subsystem, Lift.LiftState height, int angle) {
        liftSubsystem = subsystem;
        desiredState = height;
        yawArmAngle = angle;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.setLiftState(desiredState);
        depositState = DepositState.RAISING;
    }

    @Override
    public void execute() {
        switch (depositState) {
            case RAISING:
                if (liftSubsystem.canControlArm()) {
                    liftSubsystem.setYawArmAngle(yawArmAngle);
                    depositState = DepositState.MOVING_ARM;
                }
                break;
            case MOVING_ARM:
                if (!liftSubsystem.isYawArmBusy() && !liftSubsystem.isBusy()) {
                    liftSubsystem.openGrabber();
                    eTime.reset();
                    depositState = DepositState.DROPPING;
                }
                break;
            case DROPPING:
                if (eTime.time() > 0.5) {
                    liftSubsystem.setLiftState(Lift.LiftState.RETRACT);
                    depositState = DepositState.RETRACTING;
                }
                break;
            case RETRACTING:
                if (!liftSubsystem.isBusy() && !liftSubsystem.isYawArmBusy()) {
                    depositState = DepositState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return depositState == DepositState.IDLE;
    }
}
