package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveForwardCommand extends Command {

    CommandSwerveDrivetrain drive;
    CommandXboxController controller;

    Command thing = new WaitCommand(9);

    SwerveRequest.RobotCentric forward = new SwerveRequest.RobotCentric();
    
    public DriveForwardCommand(CommandSwerveDrivetrain drive, CommandXboxController controller) {
        this.controller = controller;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setControl(forward.withVelocityX(controller.getLeftTriggerAxis()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
