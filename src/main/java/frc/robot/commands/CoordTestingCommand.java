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
    /**
     * current state
     */
    private ScoringPos currentPos;
    /**
     * Forcefully go to the given state
     */
    private boolean forcePath;

    /* ----- Initilization ----- */

    public CoordTestingCommand(ScoringPos pos) {
        this.targetPos = pos;
        currentPos = CT.getPos();
    }

    @Override
    public void initialize() {

        currentPos = CT.getPos(); 
        boolean validPath = false;
        Set<ScoringPos> connectedPathsSet = CT.allowedPaths.get(currentPos);
        if (connectedPathsSet.contains(targetPos)) {
            CT.setPos(targetPos);
            validPath = true;
            if (debugging.CoordAllowedPathsDebugging) {
                validPath = false;
                System.out.println("Invalid Path");
            }
        }
        if (debugging.CoordAllowedPathsDebugging) {
            SmartDashboard.putBoolean("Reefscape/Debugging/Coordination/Valid Path?", validPath);
        }
        if (debugging.CoordAllowedPathsDebugging) {
            SmartDashboard.putString("Reefscape/Debugging/Paths", currentPos + "");
            SmartDashboard.putString("Reefscape/Debugging/Paths", connectedPathsSet.toString());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
