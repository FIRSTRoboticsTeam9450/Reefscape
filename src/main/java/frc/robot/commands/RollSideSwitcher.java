package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;

/*
 * Roll wrist to opposite side
 */
public class RollSideSwitcher extends Command{
    
    CoordinationSubsytem CT = CoordinationSubsytem.getInstance();

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
