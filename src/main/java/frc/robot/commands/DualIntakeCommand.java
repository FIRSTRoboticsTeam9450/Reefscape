package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DualIntakeSubsystem;

public class DualIntakeCommand extends Command{

    /* ----- Instance of the subsystem ----- */
    private DualIntakeSubsystem DI = DualIntakeSubsystem.getInstance();

    /* ----- Laser Can ----- */
    //The distance for the LaserCan to say it sees something
    //1inch = 25.4mm
    private double coralTriggerDistance = 10; //unit is mm
    private double algaeTriggerDistance = 25.4; //units is mm

    private boolean algae;

    private boolean finished;

    public DualIntakeCommand(boolean algae) {
        this.algae = algae;
    }

    /* ----- Initialization ----- */

    @Override
    public void initialize() {
        finished = false;
        addRequirements(DI);
        if (algae && DI.getAlgaeLaserDistance() > algaeTriggerDistance) {
            DI.setVoltage(-5);
        } else if (!algae && DI.getCoralLaserDistance() > coralTriggerDistance) {
            DI.setVoltage(5);
        }
    }

    /* ----- Updaters ----- */

    @Override
    public void execute() {
        if (algae) {
            if (DI.getAlgaeLaserDistance() < 10) {
                DI.setVoltage(-5);
                finished = true;
            }
        } else {
            if (DI.getCoralLaserDistance() < coralTriggerDistance) {
                DI.setVoltage(0);
                finished = true;
            }
        }
        
    }

    /* ----- Finishers ----- */

    @Override
    public boolean isFinished() {
        return finished;
        // return (DI.getCoralLaserDistance() < coralTriggerDistance || DI.getAlgaeLaserDistance() < algaeTriggerDistance);
    }

    @Override
    public void end(boolean interrupted) {
        // DI.setVoltage(0);
    }
    
}
