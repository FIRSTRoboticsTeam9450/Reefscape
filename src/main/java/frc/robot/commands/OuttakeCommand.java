package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DualIntakeSubsystem;

/**
 * will slowly outtake depending if holding onto a coral or algae, runs for 1 second
 */
public class OuttakeCommand extends Command {

    /* ----- Subsystem Instance ----- */
    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    /* ----- Variables ----- */
    private Timer timer = new Timer();

    /* ----------- Initialization ----------- */

    public OuttakeCommand() {
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        intake.setVoltage(-2);

        timer.reset();
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        return timer.get() > 1;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }


    
}
