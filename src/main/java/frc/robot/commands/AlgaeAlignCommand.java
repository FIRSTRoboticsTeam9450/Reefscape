package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Will Align to an algae, good for picking them up (algae on ground)
 */
public class AlgaeAlignCommand extends Command {
    
    /* ----- Max Speeds ----- */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private Timer timer = new Timer();

    /* ----- Subsystem Instances ----- */
    private LimelightSubsystem limelight = LimelightSubsystem.getInstance();
    private DualIntakeSubsystem intake = new DualIntakeSubsystem();

    /* ----- PID's ----- */
    private PIDController pid = new PIDController(0.01, 0, 0); //0.015, 0, 0.01
    private PIDController pidRotation = new PIDController(0.015, 0, 0); //0.015, 0, 0.005
    private PIDController pidForward = new PIDController(0.01, 0, 0); //0.035, 0, 0.01

    /* ----- Variables ----- */
    private double target;
    private CommandSwerveDrivetrain drive;
    private double power = 0;
    private double rotationPower = 0;
    private boolean atTarget;

    /* ----- Swerve Drive ----- */
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* ----------- Initialization ----------- */

    /**
     * Constructor
     * @param drive instance of CommandSwerveDrive
     * @param target PID target/setpoint
     */
    public AlgaeAlignCommand(CommandSwerveDrivetrain drive, double target) {
        this.target = target;
        this.drive = drive;
        pid.setSetpoint(target);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.restart();
        atTarget = false;
        limelight.setAlgaeMode(true);
        SmartDashboard.putBoolean("Align command running?", true);
    }

    /* ----------- Updaters ----------- */

    // x = 4
    // y = 5.2
    // yaw -60
    @Override
    public void execute() {
        double tx = limelight.getTx();
        if (Math.abs(tx - target) < 1) {
            atTarget = true;
        }

        if (!atTarget) {
            power = pid.calculate(tx);
            SwerveRequest driveAlign = driveRequest.withVelocityY(-MathUtil.clamp(power, -0.15, 0.15) * MaxSpeed).withVelocityX(0);
            drive.setControl(driveAlign);
            SmartDashboard.putNumber("Power", power);
        } else {
            SwerveRequest driveAlign = driveRequest.withVelocityX(0.5).withVelocityY(0);
            drive.setControl(driveAlign);
        }

    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        return intake.getCoralLaserDistance() < 12 || timer.get() > 4;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setAlgaeMode(false);
        SmartDashboard.putBoolean("Align command running?", false);
    }

}
