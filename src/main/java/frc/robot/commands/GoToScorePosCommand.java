package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;

public class GoToScorePosCommand extends Command {

    CoordinationSubsytem score = CoordinationSubsytem.getInstance();

    @Override
    public void initialize() {
        if (score.getDesiredLevel() == 4 && !Constants.l4mode && !score.getAlgae()) {
            new CoordinationCommand(ScoringPos.PRE_L4).schedule();
        } else {
            new CoordinationCommand(ScoringPos.GO_SCORE_CORAL).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
