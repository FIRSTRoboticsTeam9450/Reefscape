package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordTestingSubsystem;
import frc.robot.subsystems.DiffWristSubsystem;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.subsystems.ElbowSubsystem;

public class ScoringCommand extends Command{ // DONT USE
    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    private ElbowSubsystem elbow = ElbowSubsystem.getInstance();
    CoordTestingCommand score = new CoordTestingCommand(ScoringPos.SCORE_CORAL);
    CoordTestingCommand elev = new CoordTestingCommand(ScoringPos.ScoreL4);
    CoordTestingCommand store = new CoordTestingCommand(ScoringPos.CORAL_STORE);
    private DiffWristSubsystem wrist = DiffWristSubsystem.getInstance();
    CoordTestingSubsystem scoreSub = CoordTestingSubsystem.getInstance();

    Timer timer = new Timer();

    ScoringPos position;

    public ScoringCommand() {
    }

    @Override
    public void initialize() {
        position = scoreSub.getPos();
        //score.schedule();
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
        // if(elbow.atSetpoint() && wrist.atPitchSetpoint()) {
        //     return true;
        // }
        // return false;
        return timer.get() > 2;
    }
    @Override
    public void end(boolean interrupted) {
        // System.out.println("GOT HERE ______________");
        intake.setVoltage(0);
        store.schedule();
        
    }
}
