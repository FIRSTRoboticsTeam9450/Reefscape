package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.IntakeIDS;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DualIntakeSubsystem;

/**
 * Will Align to an algae, good for picking them up (algae on ground)
 */
public class AlgaeAlignCommand extends Command {
    
    /* ----- Max Speeds ----- */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private Timer timer = new Timer();

    //Limelight
    //LimelightSubsystem limelight = LimelightSubsystem.getInstance();
    DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    /* ----- PID's ----- */
    private PIDController pid = new PIDController(0.03, 0, 0); //0.015, 0, 0.01
    private PIDController pidRotation = new PIDController(0.015, 0, 0); //0.015, 0, 0.005
    private PIDController pidForward = new PIDController(0.05, 0, 0); //0.035, 0, 0.01

    /* ----- Variables ----- */
    private double target;
    private double taTarget = 10.3;
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
        this.target = -18;
        this.drive = drive;
        pid.setSetpoint(-18);
        pidForward.setSetpoint(taTarget);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.restart();
        atTarget = false;
    }

    /* ----------- Updaters ----------- */

    // x = 4
    // y = 5.2
    // yaw -60
    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX("limelight-coral");
        double ta = LimelightHelpers.getTA("limelight-coral");
        
        pid.setSetpoint(-Math.sqrt(25 * ta) - 1);
        
        if (Math.abs(pid.getError()) < 1 && Math.abs(pidForward.getError()) < 2) {
            atTarget = true;
        }
        //pid.setSetpoint(-0.75 * ta - 11);

        power = pid.calculate(tx);
        double powerForward = pidForward.calculate(ta);
        if (atTarget) {
            SwerveRequest driveForward = driveRequest.withVelocityY(0).withVelocityX(0.35 * MaxSpeed);
            drive.setControl(driveForward);
        } else {
            SwerveRequest driveAlign = driveRequest.withVelocityY(MathUtil.clamp(power, -0.35, 0.35) * MaxSpeed).withVelocityX(MathUtil.clamp(powerForward, -0.25, 0.25) * MaxSpeed);
            drive.setControl(driveAlign);
        }
        

    }

    /* ----------- Finishers ----------- */

    @Override
    public boolean isFinished() {
        return intake.hasCoral() || timer.get() > 2;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(driveRequest.withVelocityX(0).withVelocityY(0));
    }

}
