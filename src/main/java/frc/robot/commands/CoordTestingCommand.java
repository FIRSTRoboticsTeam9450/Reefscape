package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.debugging;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordTestingSubsystem;

/**
 * Uses Logic to see if the state you want to go to is an allowed state to go to.
 * Ex: can't go from coral Intake to Algae intake.
 */
public class CoordTestingCommand extends Command {

    /* ----- Fields ----- */

    /**
     * Instance of the Coordintation subsystem that is used
     */
    private CoordTestingSubsystem CT = CoordTestingSubsystem.getInstance();
    
    /**
     * Target Pos
     */
    private ScoringPos targetPos;
    private ScoringPos currentPos;

    /* ----- Initilization ----- */

    public CoordTestingCommand(ScoringPos pos) {
        this.targetPos = pos;
        currentPos = CT.getPos();
    }

    @Override
    public void initialize() {
        currentPos = CT.getPos(); 
        boolean temp;
        Set<ScoringPos> testingSet = CT.allowedPaths.get(currentPos);
        if (testingSet.contains(targetPos)) {
            CT.setPos(targetPos);
            temp = true;
        } else {
            if (debugging.CoordAllowedPathsDebugging) {
                temp = false;
                System.out.println("Invalid Path");
            }
        }
        if (debugging.CoordAllowedPathsDebugging) {
            SmartDashboard.putBoolean("Reefscape/Coordination/Valid Path?", temp);
        }
        if (debugging.CoordAllowedPathsDebugging) {
            System.out.println(testingSet);
            System.out.println(targetPos);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
