package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignCommand extends Command {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    LimelightSubsystem limelight = LimelightSubsystem.getInstance();

    double target;
    CommandSwerveDrivetrain drive;

    PIDController pid = new PIDController(0.04, 0, 0);

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public AlignCommand(CommandSwerveDrivetrain drive, double target) {
        this.target = target;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double tx = limelight.getTx();
        double power = pid.calculate(tx);
        power = MathUtil.clamp(power, -0.1, 0.1);
        SwerveRequest.FieldCentric driveRequestAlign = driveRequest.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(0.1 * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(0 * MaxAngularRate); // Drive counterclockwise with negative X (left)
        drive.applyRequest(() -> driveRequestAlign);

        SmartDashboard.putNumber("Power", power);
    }



}
