package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElbowSubsystem;

/**
 * Command for the Elbow
 * The Elbow is the Part the connects the elevator and the differential wrist
 */
public class ElbowCommand extends Command{
    
    /* ----- Fields ----- */

    /**
     * Instance of the elbow subsystem
     */
    private ElbowSubsystem elbow = ElbowSubsystem.getInstance();

    /**
     * Setpoint to go to
    */
    private double setpoint;

    /* ----- Initialization ----- */

    /**
     * Constructor for the Elbow command
     * @param setpoint target position(Angle) of the Elbow (0 degrees is level)
     */
    public ElbowCommand(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        addRequirements(elbow);
        elbow.setSetpoint(setpoint);
    }

    /* ----- Finishers ----- */

    @Override
    public boolean isFinished() {
        return true;
    }

}
