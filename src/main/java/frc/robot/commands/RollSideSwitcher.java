package frc.robot.commands;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DiffWristSubsystem;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.subsystems.ElbowSubsystem;

/*
 * Roll wrist to opposite side
 */
public class RollSideSwitcher extends Command{
    
    /* ----- Subsystem Instance ----- */
    private CoordinationSubsytem CT = CoordinationSubsytem.getInstance();
    private DiffWristSubsystem wrist = DiffWristSubsystem.getInstance();
    private DualIntakeSubsystem Intake = DualIntakeSubsystem.getInstance();

    boolean finished = false;
    int count = 0;
    private boolean hasAlgae;

    /* ----------- Initialization ----------- */

    public RollSideSwitcher() {

    }

    @Override
    public void initialize() {
        finished = false;
        count = 0;
        hasAlgae = Intake.hasAlgae();
    }

    @Override
    public void execute() {
        if (!hasAlgae) {
            if (CT.getAllAtSetpoints() && CT.getPos() == ScoringPos.CORAL_STORE || (CT.getPos() == ScoringPos.GO_SCORE_CORAL && CT.getScoringLevel() == 4)) {
                if (!finished)
                    CT.rollToOtherSide();
                finished = true;
            } else if (CT.getPos() == ScoringPos.GO_SCORE_CORAL && CT.getScoringLevel() != 1) {
                wrist.setPitchSetpoint(-150);
                if (wrist.atPitchSetpoint() && !finished) {
                    CT.rollToOtherSide();
                    finished = true;
                }
            }
        }
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        if (hasAlgae) {
            return true;
        }
        if (finished && wrist.atRollSetpoint()) {
            count++;
        }
        return count > 2;
    }

    @Override
    public void end(boolean interrupted) {
        if (CT.getPos() == ScoringPos.GO_SCORE_CORAL && (CT.getScoringLevel() == 3 || CT.getScoringLevel() == 2) && !interrupted) {
            new CoordinationCommand(ScoringPos.GO_SCORE_CORAL).schedule();
        }
    }

}
