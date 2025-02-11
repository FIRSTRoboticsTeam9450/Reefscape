package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command that deals with the elevator subsystem
 */
public class ElevatorCommand extends Command {
    
    /* ----- Subsystem Instance ----- */
    private ElevatorSubsystem elev = ElevatorSubsystem.getInstance();

    /* ----- Variables ----- */
    private double setpoint;

    /* ----------- Initialization ----------- */

    /**
     * Constructer of the Elevator command
     * @param setpoint The position(motor revolutions) of where the elevator should go
     */
    public ElevatorCommand(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * sets the elevator's setpoint to the previously given setpoint
     */
    @Override
    public void initialize() {
        addRequirements(elev);
        elev.setSetpoint(setpoint);
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        return true;
    }

}
