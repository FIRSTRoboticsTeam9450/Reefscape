package frc.robot.commands;


import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotConstants.*;
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
    private final CoordinationCommand retry = new CoordinationCommand(ScoringPos.GO_TO_SCORE);
    private final CoordinationCommand score = new CoordinationCommand(ScoringPos.CORAL_SCORE);
    private final CoordinationCommand elev = new CoordinationCommand(ScoringPos.CORAL_SCORE_L4);
    private final SequentialCommandGroup elevAndWait = new SequentialCommandGroup(new WaitCommand(0.2).andThen(new CoordinationCommand(ScoringPos.CORAL_SCORE_L4).andThen(new WaitCommand(0.1))));
    private final CoordinationCommand store = new CoordinationCommand(ScoringPos.CORAL_STORE);
    private final SequentialCommandGroup outtakeThenStop = new SequentialCommandGroup(new InstantCommand(() -> intake.setVoltage(-1.5)).andThen(new WaitCommand(0.44).andThen(new InstantCommand(() -> intake.setVoltage(0)))));

    // ----- Variables -----
    private final Timer timer = new Timer();
    private ScoringPos position;
    private boolean algae;
    private double runDelay;
    private boolean running;

    /** Initialization logic based on current scoring position. */
    @Override
    public void initialize() {
        runDelay = 0;
        algae = scoreSub.getAlgae();
        position = scoreSub.getPos();

        if (position != ScoringPos.GO_TO_SCORE && !DriverStation.isAutonomous()) {
            new CoordinationCommand(ScoringPos.GO_TO_SCORE).schedule();
            running = true;
        } else {
            score(); // Direct scoring if already in correct state
        }
    }

    /** Handles the actual scoring based on detected conditions. */
    public void score() {
        if (algae || position == ScoringPos.ALGAE_STORE) {
            intake.setVoltage(scoreSub.getAlgaeNet() ? -5 : -2);
        } else if (scoreSub.getDesiredLevel() == 4) {
            elevAndWait.schedule();
            intake.setVoltage(0.5);
        } else if (scoreSub.getDesiredLevel() == 1 || scoreSub.getDesiredLevel() == 0) {
            switch (scoreSub.getDesiredLevel()) {
                case 0:
                    intake.setVoltage(-4.125);
                    break;
                case 1:
                    intake.setVoltage(-3);
                    break;
                default:
                    intake.setVoltage(-3);
                    break;
            }
        } else {
            if (scoreSub.getDesiredLevel() == 2) {
                // intake.setVoltage(-0.5);
            }
            score.schedule();
            intake.setVoltage(0);
        }
        timer.restart();
    }

    /** Main execution logic - monitors subsystem state before initiating score. */
    @Override
    public void execute() {
        if (runDelay > 20 || scoreSub.getDesiredLevel() != 4) {
            if (running && scoreSub.getAllAtSetpoints()) {
                score();
                running = false;
            }
        } else {
            scoreSub.checkAllAtSetpoints();
            runDelay++;
        }
    }

    /** Determines if the command has completed its scoring cycle. */
    @Override
    public boolean isFinished() {
        if (running) return false;

        double timeElapsed = timer.get();
        return DriverStation.isAutonomous() || algae ? timeElapsed > 0.5 : scoreSub.getDesiredLevel() == 4 ? timeElapsed > 1.45 : timeElapsed > 0.8;
    }

    /** Logic to run at command end - retries or transitions to storage depending on state. */
    @Override
    public void end(boolean interrupted) {
        if (scoreSub.getDesiredLevel() != 2) {
            intake.setVoltage(0);
        } else {
            // outtakeThenStop.schedule();
            intake.setVoltage(0);
        }

        if (intake.hasCoral() && !DriverStation.isAutonomous()) {
            retry.schedule();
        } else if (!algae && CoordinationSubsytem.autoGround) {
            if (scoreSub.getDesiredLevel() != 1) {
                new CoordinationCommand(ScoringPos.CORAL_STORE)
                    .andThen(new WaitCommand(0.455))
                    .andThen(new CoordinationCommand(ScoringPos.CORAL_INTAKE_GROUND)
                        .andThen(new DualIntakeCommand(false))
                        .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE)))
                    .schedule();
            } else {
                new CoordinationCommand(ScoringPos.CORAL_STORE)
                .andThen(new CoordinationCommand(ScoringPos.CORAL_INTAKE_GROUND)
                    .andThen(new DualIntakeCommand(false))
                    .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE)))
                .schedule();
            }
        } else {
            store.schedule();
        }
    }
}