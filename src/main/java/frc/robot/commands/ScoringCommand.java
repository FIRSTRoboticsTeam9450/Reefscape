package frc.robot.commands;

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
    private DiffWristSubsystem wrist = DiffWristSubsystem.getInstance();
    
    public ScoringCommand() {
    }

    @Override
    public void initialize() {
        //score.schedule();
        if(CoordTestingSubsystem.L4) {
            elev.schedule();
        }
        else {
            score.schedule();
        }
        intake.setVoltage(-.5);
        
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
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        // System.out.println("GOT HERE ______________");
        //intake.setVoltage(0);
    }
}
