package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElbowSubsystem;

public class ElbowCommand extends Command{
    
    private ElbowSubsystem elbow = ElbowSubsystem.getInstance();

    private double setpoint;

    public ElbowCommand(double setpoint) {
        System.out.println("Do you do stuff");
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        addRequirements(elbow);
        elbow.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
