package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.robotConstants.*;
import frc.robot.subsystems.DualIntakeSubsystem;

public class ScoreOrIntakeCommand extends Command{
    
    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    private boolean hasCoral;

    @Override
    public void initialize() {

        boolean use1Controller = SmartDashboard.getBoolean("Experimental Keybinds choosen setting", false);
        hasCoral = intake.hasCoral();
        if (use1Controller) {
            if (hasCoral) {
                new ScoringCommand().schedule();
            } else {
                new CoordinationCommand(ScoringPos.INTAKE_CORAL)
                    .andThen(new DualIntakeCommand(false))
                    .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE)).schedule();
            }
        } else {
            new ScoringCommand().schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
