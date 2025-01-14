package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    
    private ElevatorSubsystem elev = ElevatorSubsystem.getInstance();

    private double setpoint;

    public ElevatorCommand(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        elev.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
