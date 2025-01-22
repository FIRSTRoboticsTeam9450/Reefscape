package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.debugging;
import frc.robot.Constants.testingPos;
import frc.robot.subsystems.CoordTestingSubsystem;

/**
 * Uses Logic to see if the state you want to go to is an allowed state to go to.
 * Ex: can't go from coral Intake to Algae intake.
 */
public class CoordTestingCommand extends Command {

    private CoordTestingSubsystem CT = CoordTestingSubsystem.getInstance();
    
    private testingPos targetPos;
    private testingPos currentPos;

    public CoordTestingCommand(testingPos pos) {
        this.targetPos = pos;
        currentPos = CT.getPos();
    }

    @Override
    public void initialize() {
        currentPos = CT.getPos(); 
        boolean temp;
        Set<testingPos> testingSet = CT.allowedPaths.get(currentPos);
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
