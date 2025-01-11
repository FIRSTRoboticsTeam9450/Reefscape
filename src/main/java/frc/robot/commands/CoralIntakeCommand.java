package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command{
    
    //Instance of the Coral Intake Subsystem
    private CoralIntakeSubsystem CI = CoralIntakeSubsystem.getInstance();


    /* ----- Constructors ----- */

    public CoralIntakeCommand(double leftVoltage, double rightVoltage) {
    }

    public CoralIntakeCommand(double setpoint, boolean rollPID) {
    }

    /* ----- Initialization ----- */

    @Override
    public void initialize() {
    }

    /* ----- Finishers ----- */

    @Override
    public boolean isFinished() {
        return true;
    }


}