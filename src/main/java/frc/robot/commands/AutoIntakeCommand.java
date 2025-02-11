package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.DualIntakeSubsystem;

public class AutoIntakeCommand extends Command{

    /* ----- Instance of the subsystem ----- */
    private DualIntakeSubsystem DI = DualIntakeSubsystem.getInstance();

    Timer timer = new Timer();

    public AutoIntakeCommand() {
        addRequirements(DI);
    }

    /* ----- Initialization ----- */

    @Override
    public void initialize() {
        System.out.println("INTAKING");
        timer.restart();
        DI.setVoltage(12);
    }

    /* ----- Updaters ----- */

    @Override
    public void execute() {
        
    }

    /* ----- Finishers ----- */

    @Override
    public boolean isFinished() {
        return timer.get() > 2;
        // return (DI.getCoralLaserDistance() < coralTriggerDistance || DI.getAlgaeLaserDistance() < algaeTriggerDistance);
    }

    @Override
    public void end(boolean interrupted) {
        DI.setVoltage(2);
    }
    
}
