package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;

/**
 * Command to score a game piece. Automatically determines scoring logic 
 * based on coral or algae detection and scoring position/state.
 */
public class ScoringCommand extends Command {

    // ----- Subsystem Instances -----
    private final DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    private final CoordinationSubsytem scoreSub = CoordinationSubsytem.getInstance();
    private final CoordinationCommand retry = new CoordinationCommand(ScoringPos.GO_SCORE_CORAL);
    private final CoordinationCommand score = new CoordinationCommand(ScoringPos.SCORE_CORAL);
    private final SequentialCommandGroup elev = new SequentialCommandGroup(new WaitCommand(.7), new CoordinationCommand(ScoringPos.ScoreL4));
    private final CoordinationCommand store = new CoordinationCommand(ScoringPos.CORAL_STORE);
    private final InstantCommand outtake = new InstantCommand(() -> intake.setVoltage(-2));

    // ----- Variables -----
    private final Timer timer = new Timer();
    private ScoringPos position;
    private boolean algae;
    private boolean coral;
    private double runDelay;
    private boolean running;

    /** Initialization logic based on current scoring position. */
    @Override
    public void initialize() {
        runDelay = 0;
        
        algae = scoreSub.getAlgae();
        position = scoreSub.getPos();
        if(position == ScoringPos.GO_SCORE_CORAL)
        {
        }
        if (position != ScoringPos.GO_SCORE_CORAL && !DriverStation.isAutonomous()) {
            new CoordinationCommand(ScoringPos.GO_SCORE_CORAL).schedule();
            running = true;
        } else {
            score(); // Direct scoring if already in correct state
        }
        Logger.recordOutput("Reefscape/Debugging/Score/Retry?", false);
    }

    /** Handles the actual scoring based on detected conditions. */
    public void score() {
        if (algae || position == ScoringPos.ALGAE_STORE) {
            intake.setVoltage(scoreSub.getAlgaeNet() ? (DriverStation.isAutonomous() ? -12 :-10.5) : -3);
        } else if (scoreSub.getScoringLevel() == 4) {
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

    /** Main execution logic - monitors subsystem state before initiating score. */
    @Override
    public void execute() {
        if (runDelay > 2) {
            if (running && scoreSub.getAllAtSetpoints()) {
                score();
                running = false;
            }
        } else {
            scoreSub.checkAllAtSetpoints();
            runDelay++;
        }
        if (!coral) {
            outtake.schedule();
            running = false;
        }
        Logger.recordOutput("Reefscape/Debugging/Scoring Run Delay", runDelay);
    }

    /** Determines if the command has completed its scoring cycle. */
    @Override
    public boolean isFinished() {
        if (running) return false;

        double timeElapsed = timer.get();
        return DriverStation.isAutonomous() || algae ? timeElapsed > 0.5 : scoreSub.getDesiredLevel() == 4 ? timeElapsed > 1.25 :timeElapsed > 0.8;
    }

    /** Logic to run at command end - retries or transitions to storage depending on state. */
    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
        if (!DriverStation.isAutonomous()) {
            if (intake.hasCoral()) {
                Logger.recordOutput("Reefscape/Debugging/Score/Retry?", true);
                retry.schedule();
            } else if (!algae && CoordinationSubsytem.autoGround) {
                new CoordinationCommand(ScoringPos.CORAL_STORE)
                    .andThen(new WaitCommand(0.65))
                    .andThen(new CoordinationCommand(ScoringPos.INTAKE_CORAL)
                        .andThen(new DualIntakeCommand(false))
                        .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE)))
                    .schedule();
                    Logger.recordOutput("Reefscape/Debugging/Score/Retry?", false);
            } else {
                store.schedule();
                Logger.recordOutput("Reefscape/Debugging/Score/Retry?", false);
            }
        }
    }
}