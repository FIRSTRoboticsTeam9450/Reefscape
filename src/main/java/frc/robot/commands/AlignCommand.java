package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignCommand extends Command {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    LimelightSubsystem limelight = LimelightSubsystem.getInstance();

    double target;
    CommandSwerveDrivetrain drive;
    double power = 0;
    PIDController pid = new PIDController(0.01, 0, 0);

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public AlignCommand(CommandSwerveDrivetrain drive, double target) {
        this.target = target;
        this.drive = drive;
        pid.setSetpoint(target);
    }

    @Override
    public void execute() {
        double tx = limelight.getTx();
        power = pid.calculate(tx);
        SwerveRequest driveAlign = driveRequest.withVelocityY(MathUtil.clamp(power, -0.1, 0.1) * MaxSpeed);
        drive.setControl(driveAlign);
        SmartDashboard.putNumber("Power", power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
