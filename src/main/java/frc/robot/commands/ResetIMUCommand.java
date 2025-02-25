package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetIMUCommand extends Command {
    
    CommandSwerveDrivetrain drive;

    public ResetIMUCommand(CommandSwerveDrivetrain drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.seedFieldCentric();
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            RobotContainer.pigeonOffset = drive.getPigeon2().getRotation2d().getDegrees();
        } else {
            RobotContainer.pigeonOffset = drive.getPigeon2().getRotation2d().getDegrees() + 180;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
