package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;

public class DualIntakeCommand extends Command{

    /* ----- Subsystem Instance ----- */
    private DualIntakeSubsystem DI = DualIntakeSubsystem.getInstance();
    private CoordinationSubsytem score = CoordinationSubsytem.getInstance();

    /* ----- Variables ----- */
    private boolean algae;
    private boolean finished;
    private Timer algaeTimer = new Timer();

    /* ----- Command :) ----- */
    private Command wristUpReef = new CoordinationCommand(ScoringPos.GRABBED_ALGAE);
    private Command wristUpGround = new CoordinationCommand(ScoringPos.ALGAE_STORE);

    public DualIntakeCommand(boolean algae) {
        this.algae = algae;
        addRequirements(DI);
    }

    /* ----------- Initialization ----------- */

    @Override
    public void initialize() {
        finished = false;
        if (algae) {
            DI.setVoltage(12);
        } else {
            DI.setVoltage(12);
        }
    }

    /* ----------- Updaters ----------- */

    @Override
    public void execute() {
        if (finished) {
            return;
        }
        if (algae) {
            if (DI.hasAlgae()) {
                DI.setVoltage(12);
                finished = true;
                if (score.getPos() != ScoringPos.INTAKE_ALGAE) {
                    wristUpReef.schedule();
                } 
                algaeTimer.restart();
            }
        } else {
            if (DI.hasCoral()) {
                DI.setVoltage(0);
                finished = true;
            }
        }
        
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        // if (algae) {
        //     return finished && algaeTimer.get() > 0.5;
        // }
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        if (algae) {
            if (score.getPos() == ScoringPos.INTAKE_ALGAE) {
                wristUpGround.schedule();
            }
        } else {
            DI.setVoltage(0);
        }
    }
    
}
