package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignCommand extends Command {
    
    //Max Speeds
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //Limelight
    LimelightSubsystem limelight = LimelightSubsystem.getInstance();

    //Variables
    double target;
    CommandSwerveDrivetrain drive;
    double power = 0;
    double rotationPower = 0;
    PIDController pid = new PIDController(0.015, 0, 0.01);
    PIDController pidRotation = new PIDController(0.015, 0, 0.005);
    PIDController pidForward = new PIDController(0.035, 0, 0.01);

    //Swerve Drive stuff
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /* ----- Initialization ----- */

    public AlignCommand(CommandSwerveDrivetrain drive, double target) {
        this.target = target;
        this.drive = drive;
        pid.setSetpoint(target);
        pidRotation.setSetpoint(target);
        pidForward.setSetpoint(14.5);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Align command running?", true);
    }

    /* ----- Updaters ----- */

    @Override
    public void execute() {
        double tx = limelight.getTx();
        double yaw = limelight.getYaw();
        double ta = limelight.getTa();
        power = pid.calculate(tx);
        rotationPower = -pidRotation.calculate(yaw);
        double forwardPower = pidForward.calculate(ta);
        SwerveRequest driveAlign = driveRequest.withVelocityX(MathUtil.clamp(forwardPower, -0.15, 0.15) * MaxSpeed).withVelocityY(MathUtil.clamp(power, -0.15, 0.15) * MaxSpeed).withRotationalRate(MathUtil.clamp(rotationPower, -1, 1) * MaxAngularRate);
        drive.setControl(driveAlign);
        SmartDashboard.putNumber("Power", power);
    }

    /* ----- Finishers ----- */

    @Override
    public boolean isFinished() {

        if (
            ((limelight.getTx() >= -2 && limelight.getTx() <= 2) && 
            (limelight.getYaw() >= -2 && limelight.getYaw() <= 2) && 
            (limelight.getTa() >= 13)) || 
            (limelight.getTx() == 0.0 && 
            limelight.getYaw() == 0.0 && 
            limelight.getTa() == 0.0)) {

                return true;

        } else {
            return false;
        }

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Align command running?", false);
    }

}
