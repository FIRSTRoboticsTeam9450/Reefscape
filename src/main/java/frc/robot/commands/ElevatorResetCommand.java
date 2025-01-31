package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorResetCommand extends Command{
    
    private ElevatorSubsystem elev = ElevatorSubsystem.getInstance();

    public ElevatorResetCommand() {}

    @Override
    public void initialize() {
        if (!elev.getAtLimit()) {
            elev.setVoltage(-1);
        }
    }

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
