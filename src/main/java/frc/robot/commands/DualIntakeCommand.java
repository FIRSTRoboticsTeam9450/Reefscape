package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DualIntakeSubsystem;

public class DualIntakeCommand extends Command{

    /* ----- Instance of the subsystem ----- */
    private DualIntakeSubsystem DI = DualIntakeSubsystem.getInstance();

    /* ----- Laser Can ----- */
    //The distance for the LaserCan to say it sees something
    //1inch = 25.4mm
    private double coralTriggerDistance = 25.4; //unit is mm
    private double algaeTriggerDistance = 25.4; //units is mm

    /* ----- Initialization ----- */

    @Override
    public void initialize() {
        addRequirements(DI);
        if (DI.getCoralLaserDistance() > coralTriggerDistance || DI.getAlgaeLaserDistance() > algaeTriggerDistance) {
            DI.setVoltage(1);
        }
    }

    /* ----- Updaters ----- */

    @Override
    public void execute() {
        if (DI.getCoralLaserDistance() > coralTriggerDistance || DI.getAlgaeLaserDistance() > algaeTriggerDistance) {
            DI.setVoltage(1);
        }
    }

    /* ----- Finishers ----- */

    @Override
    public boolean isFinished() {
        return (DI.getCoralLaserDistance() < coralTriggerDistance || DI.getAlgaeLaserDistance() < algaeTriggerDistance);
    }

    @Override
    public void end(boolean interrupted) {
        DI.setVoltage(0);
    }
    
}
