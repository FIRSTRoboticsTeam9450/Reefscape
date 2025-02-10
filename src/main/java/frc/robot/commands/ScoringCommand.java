package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;
/*
 * Score a game piece - automatic for coral and algae
 */
public class ScoringCommand extends Command {
    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    CoordinationCommand score = new CoordinationCommand(ScoringPos.SCORE_CORAL);
    CoordinationCommand elev = new CoordinationCommand(ScoringPos.ScoreL4);
    CoordinationCommand store = new CoordinationCommand(ScoringPos.CORAL_STORE);
    CoordinationSubsytem scoreSub = CoordinationSubsytem.getInstance();

    Timer timer = new Timer();

    ScoringPos position;

    public ScoringCommand() {
    
    }

    @Override
    public void initialize() {
        position = scoreSub.getPos();
        timer.restart();
        if (position == ScoringPos.SCORE_NET || position == ScoringPos.ALGAE_STORE) {
            intake.setVoltage(5);
        } else if(scoreSub.getScoringLevel() == 4) {
            elev.schedule();
            intake.setVoltage(-.25);
        } else if (scoreSub.getScoringLevel() == 1) {
            intake.setVoltage(-1.25);
        } else {
            score.schedule();
            intake.setVoltage(-.25);
        }
        
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // finish after one second
        return timer.get() > 1;
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
        // go to store
        store.schedule();
        
    }
}
