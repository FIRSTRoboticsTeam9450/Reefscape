package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command{

    private ClimbSubsystem CS = ClimbSubsystem.getInstance();
    
    private double setpoint;
    private double maxVolts;

    public ClimbCommand(double setpoint, double maxVolts) {
        this.setpoint = setpoint;
        this.maxVolts = maxVolts;
    }

    @Override
    public void initialize() {
        CS.setSetpoint(setpoint);
        CS.setMaxVolts(maxVolts);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
