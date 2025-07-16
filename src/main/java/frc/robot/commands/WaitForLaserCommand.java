package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DualIntakeSubsystem;

/**
 * This command waits until the laser sensor detects a coral.
 * The command finishes once the intake subsystem confirms detection.
 */
public class WaitForLaserCommand extends Command {

    // // Reference to the DualIntakeSubsystem singleton
    // private final DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    // /**
    //  * Called once when the command is initially scheduled.
    //  * No initialization logic is required for this command.
    //  */
    // @Override
    // public void initialize() {
    //     // No setup needed
    // }

    // /**
    //  * Determines whether the command has completed.
    //  * @return true if the coral has been detected, false otherwise
    //  */
    // @Override
    // public boolean isFinished() {
    //     return intake.hasCoral();
    // }
}