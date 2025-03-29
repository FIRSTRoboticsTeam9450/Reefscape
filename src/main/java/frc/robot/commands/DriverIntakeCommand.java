package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;

public class DriverIntakeCommand extends Command {

    DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    CoordinationSubsytem score = CoordinationSubsytem.getInstance();

    public DriverIntakeCommand () {

    }

    @Override
    public void initialize() {
        if (!score.getAlgae()) {
            new CoordinationCommand(ScoringPos.INTAKE_CORAL).andThen(new DualIntakeCommand(false)).andThen(new CoordinationCommand(ScoringPos.CORAL_STORE)).schedule();
        } else {
            new GoToScorePosCommand().schedule();
        }
    }
    
}
