package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Controlls driving during auto, i can't say much else about it...
 */
public class DriveInAutoCommand extends Command {

    /* ----- Swerve Drive ----- */
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withVelocityX(0.5);

    /* ----------- Initialization ----------- */

    public DriveInAutoCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    /* ----------- Updaters ----------- */

    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(drivetrain.applyRequest(()-> request));
    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
