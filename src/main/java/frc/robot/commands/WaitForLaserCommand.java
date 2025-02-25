package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DualIntakeSubsystem;

public class WaitForLaserCommand extends Command {
    
    DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return intake.getCoralLaserDistance() < Constants.robotConfig.getCoralTriggerDistance();
    }

}
