package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;

/**
 * Honestly got no clue what this does, but I reckon its something to do with the robot driving field centricly
 */
public class FieldCentricCommand extends Command {

    /* ----- Subsystem Instances ----- */
    private CommandSwerveDrivetrain drive;
    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    private CoordinationSubsytem score = CoordinationSubsytem.getInstance();

    /* ----- Swerve Drive ----- */
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(RobotContainer.MaxSpeed * 0.1).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    /* ----- Variables? (I think) ----- */
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier rot;

    Timer timer = new Timer();
    boolean scored;
    boolean backUp;

    /* ----------- Initialization ----------- */
    
    public FieldCentricCommand(CommandSwerveDrivetrain drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        addRequirements(drive);
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.rot = rot;
    }

    @Override
    public void initialize() {
        scored = false;
        backUp = intake.getAlgaeLaserDistance() < 50 || intake.getCoralLaserDistance() < 50;
    }

    /* ----------- Updaters ----------- */

    @Override
    public void execute() {
        drive.setControl(
            driveRobotCentric.withVelocityX(-y.getAsDouble() * RobotContainer.MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-x.getAsDouble() * RobotContainer.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-rot.getAsDouble() * RobotContainer.MaxAngularRate) // Drive counterclockwise with negative X (left)
        );

        if (!scored) {
            if (score.getPos() == ScoringPos.ALGAEL1 || score.getPos() == ScoringPos.ALGAEL2) {
                scored = intake.getAlgaeLaserDistance() < 50;
            } else {
                scored = intake.getCoralLaserDistance() > 50 && intake.getAlgaeLaserDistance() > 50;
            }
            if (scored) {
                timer.restart();
            }
        } else if (backUp) {
            drive.setControl(driveRobotCentric.withVelocityX(0).withVelocityY(0.25).withRotationalRate(0));
        }
        
    }

    /* ------------ Finishers ----------- */

    @Override
    public boolean isFinished() {
        return scored && timer.get() > 1;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
