package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.DualIntakeSubsystem;

public class DualIntakeCommand extends Command{

    /* ----- Instance of the subsystem ----- */
    private DualIntakeSubsystem DI = DualIntakeSubsystem.getInstance();

    /* ----- Laser Can ----- */
    //The distance for the LaserCan to say it sees something
    //1inch = 25.4mm
    private double coralTriggerDistance = 60; //unit is mm
    private double algaeTriggerDistance = 25.4; //units is mm

    private boolean algae;

    private boolean finished;

    Command wristUp = new CoordTestingCommand(ScoringPos.GRABBED_ALGAE);

    public DualIntakeCommand(boolean algae) {
        this.algae = algae;
        addRequirements(DI);
    }

    /* ----- Initialization ----- */

    @Override
    public void initialize() {
        finished = false;
        if (algae && DI.getAlgaeLaserDistance() > algaeTriggerDistance) {
            DI.setVoltage(-12);
        } else if (!algae && DI.getCoralLaserDistance() > coralTriggerDistance) {
            DI.setVoltage(12);
        }
    }

    /* ----- Updaters ----- */

    @Override
    public void execute() {
        if (algae) {
            if (DI.getAlgaeLaserDistance() < algaeTriggerDistance) {
                DI.setVoltage(-5);
                finished = true;
                wristUp.schedule();
            }
        } else {
            if (DI.getCoralLaserDistance() < coralTriggerDistance) {
                DI.setVoltage(2);
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
