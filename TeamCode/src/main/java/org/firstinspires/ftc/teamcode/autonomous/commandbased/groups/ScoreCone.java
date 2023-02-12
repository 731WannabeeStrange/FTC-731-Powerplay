package org.firstinspires.ftc.teamcode.autonomous.commandbased.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.DepositCone;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.FindCone;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.GoToLiftState;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.RetrieveCone;
import org.firstinspires.ftc.teamcode.autonomous.commandbased.commands.TransferCone;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;

public class ScoreCone extends SequentialCommandGroup {
    public ScoreCone(Intake intakeSubsystem, Lift liftSubsystem, int desiredAngle,
                     Lift.LiftState desiredHeight, double v4bpos) {
        addCommands(
                new ParallelCommandGroup(
                        new DepositCone(liftSubsystem, desiredHeight, desiredAngle),
                        new FindCone(intakeSubsystem, v4bpos)),
                new ParallelDeadlineGroup(
                        new RetrieveCone(intakeSubsystem),
                        new GoToLiftState(liftSubsystem, Lift.LiftState.RETRACT)),
                new TransferCone(intakeSubsystem, liftSubsystem));
    }
}
