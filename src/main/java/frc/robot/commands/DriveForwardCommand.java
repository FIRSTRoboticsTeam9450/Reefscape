package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DualIntakeSubsystem;

public class DriveForwardCommand extends Command {

    CommandSwerveDrivetrain drive;
    CommandXboxController controller;
    DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    // Command thing = new WaitCommand(9);

    private Timer timer = new Timer();

    SwerveRequest.RobotCentric forward = new SwerveRequest.RobotCentric();
    SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public DriveForwardCommand(CommandSwerveDrivetrain drive, CommandXboxController controller) {
        this.controller = controller;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        drive.setControl(forward.withVelocityX(0.75));
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 3 || intake.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(fieldCentric);
    }
    
}
