package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.DualIntakeSubsystem;

public class AutoIntakeCommand extends Command{

    /* ----- Instance of the subsystem ----- */
    private DualIntakeSubsystem DI = DualIntakeSubsystem.getInstance();
    private Command store = new CoordinationCommand(ScoringPos.START);

    Timer timer = new Timer();
    Timer grabbedTimer = new Timer();

    boolean grabbed = false;
    boolean source;

    public AutoIntakeCommand(boolean source) {
        this.source = source;
        addRequirements(DI);
    }

    /* ----- Initialization ----- */

    @Override
    public void initialize() {
        //System.out.println("INTAKING");
        timer.restart();
        DI.setVoltage(12);
        grabbed = false;
    }

    /* ----- Updaters ----- */

    @Override
    public void execute() {
        if (DI.hasCoral() && !grabbed) {
            grabbedTimer.restart();
            grabbed = true;
        }
        
    }

    /* ----- Finishers ----- */

    @Override
    public boolean isFinished() {
        if (source) {
            return DI.hasCoral() || timer.get() > 3;
        }
        return timer.get() > 5 || (DI.hasCoral() && grabbedTimer.get() > 0.3);

        // return (DI.getCoralLaserDistance() < coralTriggerDistance || DI.getAlgaeLaserDistance() < algaeTriggerDistance);
    }

    @Override
    public void end(boolean interrupted) {
        DI.setVoltage(6);
        new WaitCommand(0.25).andThen(new InstantCommand(() -> DI.setVoltage(0.5))).schedule();
        System.out.println("SLOW INTAKE SAD NO WHY ACTUALLY SOBBING :'(");
        //store.schedule();
    }
    
}
