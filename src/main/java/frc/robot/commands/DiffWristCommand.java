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
    private double leftVoltage;
    private double rightVoltage;
    private double setpoint;
    private boolean pitchPID = false;

    /* ----------- Initialization ----------- */

    /**
     * Sets the voltage of both motors on the Differental Wrist
     * @param leftVoltage voltage for the left motor to be set to
     * @param rightVoltage Voltage for the right motor to be set to
     */
    public DiffWristCommand(double leftVoltage, double rightVoltage) {
        this.leftVoltage = leftVoltage;
        this.rightVoltage = rightVoltage;
    }

    /**
     * Will set one of the two PIDs on the Different wrist to the given setpoint
     * @param setpoint setpoint to go to
     * @param pitchPID True if for the roll PID, false if for the Yaw pid
     */
    public DiffWristCommand(double setpoint, boolean pitchPID) {
        this.setpoint = setpoint;
        this.pitchPID = pitchPID;
    }

    @Override
    public void initialize() {
        addRequirements(DW);
        boolean runPID = DW.getIfDoingPIDS();
        if (runPID) {
            if (pitchPID) {
                DW.setPitchSetpoint(setpoint);
            } else {
                DW.setRollSetpoint(setpoint);
            }
        } else {
            DW.setVoltage(leftVoltage, rightVoltage);
        }
    }

    /* ----------- Finishers ----------- */
    
    @Override
    public boolean isFinished() {
        return true;
    }

}
