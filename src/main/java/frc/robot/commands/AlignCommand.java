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
import frc.robot.LimelightHelpers;
import frc.robot.Constants.IntakeIDS;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignCommand extends Command {
    
    //Max Speeds
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    Timer timer = new Timer();

    //Limelight
    //LimelightSubsystem limelight = LimelightSubsystem.getInstance();
    DualIntakeSubsystem intake = new DualIntakeSubsystem();

    //Variables
    double target;
    CommandSwerveDrivetrain drive;
    double power = 0;
    double rotationPower = 0;
    PIDController pid = new PIDController(0.01, 0, 0); //0.015, 0, 0.01
    PIDController pidRotation = new PIDController(0.015, 0, 0); //0.015, 0, 0.005
    PIDController pidForward = new PIDController(0.01, 0, 0); //0.035, 0, 0.01

    boolean atTarget;

    //Swerve Drive stuff
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* ----- Initialization ----- */

    public AlignCommand(CommandSwerveDrivetrain drive, double target) {
        this.target = target;
        this.drive = drive;
        pid.setSetpoint(target);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.restart();
        atTarget = false;
        LimelightHelpers.setPipelineIndex("limelight", 1);
        SmartDashboard.putBoolean("Align command running?", true);
    }

    /* ----- Updaters ----- */
    // x = 4
    // y = 5.2
    // yaw -60
    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX("limelight");
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

    /* ----- Finishers ----- */

    @Override
    public boolean isFinished() {
        return intake.getCoralLaserDistance() < 12 || timer.get() > 4;
        // if (
        //     ((limelight.getTx() >= -2 && limelight.getTx() <= 2) && 
        //     (limelight.getYaw() >= -2 && limelight.getYaw() <= 2) && 
        //     (limelight.getTa() >= 13)) || 
        //     (limelight.getTx() == 0.0 && 
        //     limelight.getYaw() == 0.0 && 
        //     limelight.getTa() == 0.0)) {

        //         return true;

        // } else {
        //     return false;
        // }

    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        SmartDashboard.putBoolean("Align command running?", false);
    }

}
