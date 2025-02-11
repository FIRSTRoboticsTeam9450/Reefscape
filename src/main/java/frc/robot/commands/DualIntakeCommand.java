package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.DualIntakeSubsystem;

public class DualIntakeCommand extends Command{

    /* ----- Subsystem Instance ----- */
    private DualIntakeSubsystem DI = DualIntakeSubsystem.getInstance();

    /* ----- Laser Can ----- */
    //The distance for the LaserCan to say it sees something
    //1inch = 25.4mm
    private double coralTriggerDistance = 15; //unit is mm
    private double algaeTriggerDistance = 25.4; //units is mm

    /* ----- Variables ----- */
    private boolean algae;
    private boolean finished;

    /* ----- Command :) ----- */
    private Command wristUp = new CoordinationCommand(ScoringPos.GRABBED_ALGAE);

    public DualIntakeCommand(boolean algae) {
        this.algae = algae;
        addRequirements(DI);
    }

    /* ----------- Initialization ----------- */

    @Override
    public void initialize() {
        finished = false;
        if (algae && DI.getAlgaeLaserDistance() > algaeTriggerDistance) {
            DI.setVoltage(-5);
        } else if (!algae && DI.getCoralLaserDistance() > coralTriggerDistance) {
            DI.setVoltage(12);
        }
    }

    /* ----------- Updaters ----------- */

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
                DI.setVoltage(0.5);
                finished = true;
            }
        }
        
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
    }
    
}
