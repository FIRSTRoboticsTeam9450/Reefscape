package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;

public class CancelCommand extends Command {

    CoordinationSubsytem score = CoordinationSubsytem.getInstance();
    DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    Command stop = new SequentialCommandGroup(
        new CoordinationCommand(ScoringPos.CORAL_STORE), 
        new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
    );

    @Override
    public void initialize() {
        CoordinationCommand.justCancelled = true;
        if (score.getPos() == ScoringPos.INTAKE_CORAL) {
            intake.setVoltage(5);
        } else {
            intake.setVoltage(0);
        }
        stop.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
