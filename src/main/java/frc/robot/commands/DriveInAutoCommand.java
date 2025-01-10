package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveInAutoCommand extends Command {

    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withVelocityX(0.5);

    public DriveInAutoCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(drivetrain.applyRequest(()-> request));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
