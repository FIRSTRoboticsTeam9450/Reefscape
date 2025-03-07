package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;

/*
 * Score a game piece - automatic for coral and algae
 */
public class ScoringCommand extends Command {

    /* ----- Subsystem Instances ----- */
    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    private CoordinationCommand retry = new CoordinationCommand(ScoringPos.GO_SCORE_CORAL);
    private CoordinationCommand score = new CoordinationCommand(ScoringPos.SCORE_CORAL);
    private CoordinationCommand elev = new CoordinationCommand(ScoringPos.ScoreL4);
    private CoordinationCommand store = new CoordinationCommand(ScoringPos.CORAL_STORE);
    private CoordinationSubsytem scoreSub = CoordinationSubsytem.getInstance();

    /* ----- Variables ----- */
    private Timer timer = new Timer();
    private ScoringPos position;
    private boolean algae;
    private double coralTriggerDistance = Constants.robotConfig.getCoralTriggerDistance();

    /* ----------- Initialization ----------- */

    @Override
    public void initialize() {
        algae = scoreSub.getAlgae();
        position = scoreSub.getPos();
        timer.restart();
        if (scoreSub.getAlgae() || position == ScoringPos.ALGAE_STORE) {
            intake.setVoltage(12);
        } else if(scoreSub.getScoringLevel() == 4) {
            elev.schedule();
            intake.setVoltage(1);
        } else if (scoreSub.getScoringLevel() == 1) {
            intake.setVoltage(-1);
        } else {
            score.schedule();
            // intake.setVoltage(-.25);
        }
        
    }

    @Override
    public void execute() {
        // if (scoreSub.getScoringLevel() == 4 && timer.get() > 0.3) {
        //     intake.setVoltage(-2);
        // }
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        // finish after one second
        if (DriverStation.isAutonomous()) {
            return timer.get() > 0.5;
        } else {
            if (algae) {
                return timer.get() > 0.5;
            } else {
                return timer.get() > 1;
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        if (intake.hasCoral() && !DriverStation.isAutonomous()) {
            retry.schedule();
        } else {
            // go to store
            intake.setVoltage(0);
            store.schedule();
        }
        
    }
}
