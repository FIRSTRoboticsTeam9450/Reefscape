package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ThingOneConfig;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DualIntakeSubsystem;

/**
 * This command waits until the laser sensor detects a coral.
 * The command finishes once the intake subsystem confirms detection.
 */
public class SetClimberVoltageCommand extends Command {
    double voltage = 0;

    public SetClimberVoltageCommand(double voltage) {
        this.voltage = voltage;     

    }

    // Reference to the DualIntakeSubsystem singleton
    private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
    
    /**
     * Called once when the command is initially scheduled.
     * No initialization logic is required for this command.
     */
    @Override
    public void initialize() {
        climbSubsystem.setVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        voltage = 0;
        climbSubsystem.setVoltage(voltage);
        
    }

    /**
     * Determines whether the command has completed.
     * @return true if the coral has been detected, false otherwise
     */
    @Override
    public boolean isFinished() {
        return false;

    }
}