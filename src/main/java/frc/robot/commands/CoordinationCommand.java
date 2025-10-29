package frc.robot.commands;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.debugging;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Uses Logic to see if the state you want to go to is an allowed state to go to.
 * Ex: can't go from coral Intake to Algae intake.
 */
public class CoordinationCommand extends Command {

    /* ----- Subsystem Instance ----- */
    private CoordinationSubsytem CT = CoordinationSubsytem.getInstance();

    /* ----- Variables ----- */
    private ScoringPos targetPos;
    private ScoringPos currentPos;
    private boolean mode;
    public static boolean justCancelled;

    ElevatorSubsystem elev = ElevatorSubsystem.getInstance();

    /* ----------- Initilization ----------- */

    public CoordinationCommand(ScoringPos pos) {
        this.targetPos = pos;
        currentPos = CT.getPos();
        this.mode = false;
    }

    public CoordinationCommand(ScoringPos pos, boolean mode) {
        this.targetPos = pos;
        currentPos = CT.getPos();
        this.mode = mode;
    }

    @Override
    public void initialize() {
        currentPos = CT.getPos(); 
        boolean validPath = false;
        Set<ScoringPos> connectedPathsSet = CT.allowedPaths.get(currentPos);
        if(!mode) {
            if (justCancelled) {
                if (CT.getAllAtSetpoints()) {
                    justCancelled = false;
                }
            }
            if (connectedPathsSet.contains(targetPos) && !justCancelled) {
        
                CT.setPosition(targetPos);
                validPath = true;
            }
            if (debugging.CoordAllowedPathsDebugging) {
                Logger.recordOutput("Reefscape/Debugging/Coordination/Valid Path?", validPath);
                Logger.recordOutput("Reefscape/Debugging/Coordination/Trying to go to", targetPos);
            }
            if (debugging.CoordAllowedPathsDebugging) {
                Logger.recordOutput("Reefscape/Debugging/Paths", currentPos + "");
                Logger.recordOutput("Reefscape/Debugging/Paths", connectedPathsSet.toString());
            }
        } 
        else {
            CT.setPosition(targetPos);
        }
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        return true;
    }

}
