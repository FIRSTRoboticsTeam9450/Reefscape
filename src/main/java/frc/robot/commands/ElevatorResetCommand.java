package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Makes the elevator go down slowly till limit switch triggers, then it resets motor's relative encoders
 */
public class ElevatorResetCommand extends Command{
    
    /* ----- Subystem Instance ----- */
    private ElevatorSubsystem elev = ElevatorSubsystem.getInstance();

    /* ----------- Initialization ----------- */

    @Override
    public void initialize() {
        if (!elev.getAtLimit()) {
            elev.setVoltage(-1);
        }
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        return elev.getAtLimit();
    }

    @Override
    public void end(boolean interrupted) {
        elev.setVoltage(0);
        elev.setAtLimit();
    }

}
