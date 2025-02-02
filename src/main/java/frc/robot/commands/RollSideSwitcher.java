package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordTestingSubsystem;

public class RollSideSwitcher extends Command{
    
    CoordTestingSubsystem CT = CoordTestingSubsystem.getInstance();

    public RollSideSwitcher() {

    }

    @Override
    public void initialize() {
        if (CT.getAllAtSetpoints() && CT.getPos() == ScoringPos.CORAL_STORE) {
            CT.rollToOtherSide();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
