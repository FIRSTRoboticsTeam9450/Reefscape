package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DiffWristSubsystem;

public class DiffWristCommand extends Command {

    private DiffWristSubsystem DW = DiffWristSubsystem.getInstance();

    private double leftVoltage;
    private double rightVoltage;
    private double setpoint;
    private boolean rollPID = false;


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
     * @param rollPID True if for the roll PID, false if for the Yaw pid
     */
    public DiffWristCommand(double setpoint, boolean rollPID) {
        this.setpoint = setpoint;
        this.rollPID = rollPID;
    }

    @Override
    public void initialize() {
        if (DW.runPID) {
            if (rollPID) {
                DW.setRollSetpoint(setpoint);
            } else {
                DW.setYawSetpoint(setpoint);
            }
        } else {
            DW.setVoltage(leftVoltage, rightVoltage);
        }
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }

}
