package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordTestingSubsystem;
import frc.robot.subsystems.DualIntakeSubsystem;

public class FieldCentricCommand extends Command {

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(RobotContainer.MaxSpeed * 0.1).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier rot;

    CommandSwerveDrivetrain drive;
    DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    CoordTestingSubsystem score = CoordTestingSubsystem.getInstance();
    
    public FieldCentricCommand(CommandSwerveDrivetrain drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        addRequirements(drive);
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.rot = rot;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drive.setControl(
            driveRobotCentric.withVelocityX(-y.getAsDouble() * RobotContainer.MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-x.getAsDouble() * RobotContainer.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-rot.getAsDouble() * RobotContainer.MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        if (score.getPos() == ScoringPos.ALGAEL1 || score.getPos() == ScoringPos.ALGAEL2) {
            return intake.getAlgaeLaserDistance() < 50;
        } else {
            return intake.getCoralLaserDistance() > 50 && intake.getAlgaeLaserDistance() > 50;
        }
    }
}
