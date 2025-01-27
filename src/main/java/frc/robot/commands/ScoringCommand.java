package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.subsystems.ElbowSubsystem;

public class ScoringCommand extends Command{ // DONT USE
    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    private ElbowSubsystem elbow = new ElbowSubsystem();
    public ScoringCommand() {
    }

    @Override
    public void execute() {
        CoordTestingCommand score = new CoordTestingCommand(ScoringPos.SCORE_CORAL);
        if(elbow.atSetpoint()) {
            intake.setVoltage(-1);
        }
    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
