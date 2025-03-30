package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

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
    private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private boolean running;

    /* ----------- Initialization ----------- */

    @Override
    public void initialize() {
        algae = scoreSub.getAlgae();
        position = scoreSub.getPos();
        if (scoreSub.getPos() != ScoringPos.GO_SCORE_CORAL && !DriverStation.isAutonomous()) {
            new CoordinationCommand(ScoringPos.GO_SCORE_CORAL).schedule();
            running = true;
        } else {
            score();
        }
        
        
    }

    public void score() {
        if (scoreSub.getAlgae() || position == ScoringPos.ALGAE_STORE) {
            if (position == ScoringPos.SCORE_NET) {
                intake.setVoltage(-3);
            } else {
                intake.setVoltage(-10.5);
            }
        } else if(scoreSub.getScoringLevel() == 4) {
            elev.schedule();
            intake.setVoltage(0);
        } else if (scoreSub.getScoringLevel() == 1) {
            intake.setVoltage(-2);
        } else {
            score.schedule();
            intake.setVoltage(0);
        }
        timer.restart();
    }

    @Override
    public void execute() {
        if (running && scoreSub.getAllAtSetpoints()) {
            score();
            running = false;
        }
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        // finish after one second
        if (running) {
            return false;
        }
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
