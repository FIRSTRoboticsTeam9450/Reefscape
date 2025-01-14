// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.DiffWristCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
                                                            // changed to .6, originaly 1.5
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driver2 = new CommandXboxController(0);

    private final CommandXboxController m_driver1 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    private LimelightSubsystem limelight = new LimelightSubsystem();

    public RobotContainer() {

        configureBindings();

        boolean isCompetition = false;


        //if isComp... is true, it will only use auto's that name starts with "comp"
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driver2.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driver2.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driver2.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_driver2.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driver2.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driver2.getLeftY(), -m_driver2.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driver2.back().and(m_driver2.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driver2.back().and(m_driver2.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driver2.start().and(m_driver2.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driver2.start().and(m_driver2.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
            m_driver2.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        m_driver2.y().onTrue(new AlignCommand(drivetrain, 0));
        m_driver2.povUp().onTrue(new ElevatorCommand(0));
        m_driver2.povDown().onTrue(new ElevatorCommand(5));
        m_driver2.x().onTrue(new DiffWristCommand(0.55, true));
        m_driver2.b().onTrue(new DiffWristCommand(0.6, true));

        // m_driver1.a
    }

    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
    }
}
