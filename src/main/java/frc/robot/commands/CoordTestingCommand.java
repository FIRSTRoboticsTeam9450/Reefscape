package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        if (CT.allowedPaths.get(currentPos) == targetPos) {
            CT.setPos(targetPos);
        }
        System.out.println(CT.allowedPaths.get(currentPos));
        System.out.println(targetPos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
