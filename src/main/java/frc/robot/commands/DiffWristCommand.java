package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DiffWristSubsystem;

/**
 * Differential Wrist Command that deals with telling it where to go.
 */
public class DiffWristCommand extends Command {

    /* ----- Subsystem Instance ----- */
    private DiffWristSubsystem DW = DiffWristSubsystem.getInstance();

    /* ----- Variables ----- */
    private double rollSetpoint;
    private double pitchSetpoint;


    /* ----------- Initialization ----------- */

    /**
     * Sets the setpoints of both motors on the Differental Wrist
     * @param rollSetpoint setpoint for the roll to go to
     * @param pitchSetpoint setpoint for the pitch to go to
     */
    public DiffWristCommand(double rollSetpoint, double pitchSetpoint) {
        this.rollSetpoint = rollSetpoint;
        this.pitchSetpoint = pitchSetpoint;
    }

    /**
     * Will set one of the two PIDs on the Different wrist to the given setpoint
     * @param setpoint setpoint to go to
     * @param pitchPID True if for the roll PID, false if for the Pitch pid
     */


    @Override
    public void initialize() {
        addRequirements(DW);
        DW.setRollSetpoint(rollSetpoint);
        DW.setPitchSetpoint(pitchSetpoint);
    }

    /* ----------- Finishers ----------- */
    
    @Override
    public boolean isFinished() {
        return true;
    }

}
