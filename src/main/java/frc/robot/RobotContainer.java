// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlignPos;
import frc.robot.Constants.ScoringPos;
import frc.robot.commands.AlgaeAlignCommand;
import frc.robot.commands.AlignCommand2;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DiffWristCommand;
import frc.robot.commands.DualIntakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FieldCentricCommand;
import frc.robot.commands.ManualPitchCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ResetIMUCommand;
import frc.robot.commands.RollSideSwitcher;
import frc.robot.commands.ScoringCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DiffWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.commands.CoordinationCommand;;

public class RobotContainer {
    public static double MaxSpeed = 2.2; // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.7).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private static double LiftMaxSpeed = 1;
    private static double LiftMaxAngularRate = RotationsPerSecond.of(.3).in(RadiansPerSecond);

    private static double DefaultMaxSpeed = 2.2;
    private static double DefaultMaxAngularRate = RotationsPerSecond.of(0.7).in(RadiansPerSecond); // changed to .6, originaly 1.5
    
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driver2 = new CommandXboxController(1);

    private final CommandXboxController m_driver1 = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    private CoordinationSubsytem scoreSub = CoordinationSubsytem.getInstance();

    private ClimbSubsystem climb = ClimbSubsystem.getInstance();


    public static double pigeonOffset = 0;

    public RobotContainer() {
        LimelightHelpers.SetIMUMode("limelight", 1);

        configureBindings();
        registeredCommands();

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
                drive.withVelocityX(-m_driver1.getLeftY()  * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driver1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driver1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        scoreSub.setDefaultCommand(new ManualPitchCommand(() -> -m_driver2.getLeftY()));

        m_driver1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // m_driver1.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-m_driver1.getLeftY(), -m_driver1.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driver1.back().and(m_driver2.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driver1.back().and(m_driver2.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driver1.start().and(m_driver2.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driver1.start().and(m_driver2.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        drivetrain.registerTelemetry(logger::telemeterize);

        /* ----- Main Driver Keybinds ----- */
        /* Keybinds:
         * Right Trigger = Go To Scoring Pos
         * Left Trigger = Score
         * Right Bumper = Outtake
         * Left Bumper = Flip
         * X = Cancel All
         * Y = IMU Reset
         * Left Stick = Movement/Align Left
         * Right Stick = Rotate/Align Right
         * D-pad Up = Deploy Climber
         * D-pad Down = Winch Climber
         */
        
        m_driver1.rightTrigger().onTrue(new ScoringCommand());
        m_driver1.leftTrigger().onTrue(new CoordinationCommand(ScoringPos.GO_SCORE_CORAL));
        //m_driver1.leftBumper().onTrue(new CoordinationCommand(ScoringPos.ALGAEL1).andThen(new DualIntakeCommand(true)));
        m_driver1.leftBumper().onTrue(new RollSideSwitcher());
        m_driver1.rightBumper().onTrue(new OuttakeCommand());
        //m_driver1.rightBumper().onTrue(new CoordinationCommand(ScoringPos.ALGAEL2).andThen(new DualIntakeCommand(true)));
        m_driver1.x().onTrue(
            new InstantCommand(() -> intake.setVoltage(0))
            .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
            .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
            ));
        //m_driver1.a().onTrue(new CoordinationCommand(ScoringPos.SCORE_NET));
        //m_driver1.b().onTrue(new CoordinationCommand(ScoringPos.INTAKE_ALGAE).andThen(new DualIntakeCommand(true)));
        m_driver1.y().onTrue(new ResetIMUCommand(drivetrain));

        m_driver1.leftStick().whileTrue(new AlignCommand2(drivetrain, AlignPos.LEFT)).onFalse(new FieldCentricCommand(drivetrain, () -> m_driver1.getLeftX(), () -> m_driver1.getLeftY(), () -> m_driver1.getRightX()));
        m_driver1.rightStick().whileTrue(new AlignCommand2(drivetrain, AlignPos.RIGHT)).onFalse(new FieldCentricCommand(drivetrain, () -> m_driver1.getLeftX(), () -> m_driver1.getLeftY(), () -> m_driver1.getRightX()));

        //m_driver1.povRight().onTrue(new InstantCommand(() -> climb.setVoltage(4))).onFalse(new InstantCommand(() -> climb.setVoltage(0)));
        //m_driver1.povLeft().onTrue(new InstantCommand(() -> climb.setVoltage(-4))).onFalse(new InstantCommand(() -> climb.setVoltage(0)));

        m_driver1.povUp().onTrue(new ClimbCommand(0.11, 12));

        m_driver1.povDown().onTrue(new ClimbCommand(0.72, 10).andThen(new CoordinationCommand(ScoringPos.START)));

        m_driver1.povRight().onTrue(new ClimbCommand(0.92, 3));

        //m_driver1.povDown().onTrue(new ClimbCommand(0, -3));

        // m_driver1.povUp().onTrue(new ClimbCommand(-390, -3));

        /* ----- Operator Driver Keybinds ----- */
        /* Keybinds:
         * Right Trigger = Intake Coral Ground
         * Left Trigger = Set Processor Score
         * Right Bumper = Intake Coral Source
         * Left Bumper = Set Net Score
         * X = L1
         * A = L2
         * B = L3
         * Y = L4
         * D-pad Up = Intake Algae High
         * D-pad Left = Intake Algae Low
         */

        m_driver2.rightTrigger().onTrue(new CoordinationCommand(ScoringPos.INTAKE_CORAL).andThen(new DualIntakeCommand(false)).andThen(new CoordinationCommand(ScoringPos.CORAL_STORE)));
        m_driver2.leftTrigger().onTrue(new InstantCommand(() -> scoreSub.setAlgaeNet(false)));
        m_driver2.leftBumper().onTrue(new InstantCommand(() -> scoreSub.setAlgaeNet(true)));
        //m_driver2.leftTrigger().onTrue(new OuttakeCommand());
        m_driver2.rightBumper().onTrue(new CoordinationCommand(ScoringPos.INTAKE_SOURCE).andThen(new DualIntakeCommand(false).andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))));

        m_driver2.x().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(2)));
        m_driver2.a().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(1)));
        m_driver2.b().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        m_driver2.y().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(4)));
        
        m_driver2.povUp().onTrue(new CoordinationCommand(ScoringPos.ALGAEL2).andThen(new DualIntakeCommand(true)));
        m_driver2.povLeft().onTrue(new CoordinationCommand(ScoringPos.ALGAEL1).andThen(new DualIntakeCommand(true)));
        //m_driver2.povUp().onTrue(new CoordinationCommand(ScoringPos.CORAL_STORE));
        m_driver2.povDown().onTrue(new CoordinationCommand(ScoringPos.INTAKE_ALGAE).andThen(new DualIntakeCommand(true)));

        m_driver2.rightStick().onTrue(new ClimbCommand(0.075, 12));
        
        //m_driver2.rightBumper().onTrue(new CoordTestingCommand(ScoringPos.INTAKE_SOURCE).andThen(new DualIntakeCommand(false).andThen(new CoordTestingCommand(ScoringPos.CORAL_STORE))));
        // m_driver2.rightBumper().onTrue(new InstantCommand(() -> elevator.reset()));
        //m_driver2.rightBumper().onTrue(new CoordinationCommand(ScoringPos.INTAKE_SOURCE).andThen(new DualIntakeCommand(false)));
        // m_driver2.leftStick().onTrue(
        //     new InstantCommand(() -> intake.setVoltage(0))
        //     .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
        //     .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
        //     ));        
        //m_driver2.rightStick().onTrue(new InstantCommand(() -> drivetrain.runVision = true)).onFalse(new InstantCommand(() -> drivetrain.runVision = false));
        //m_driver2.leftStick().onTrue(new CoordinationCommand(ScoringPos.INTAKE_VERTICAL_CORAL).andThen(new DualIntakeCommand(false)).andThen(new CoordinationCommand(ScoringPos.CORAL_STORE)));


        //m_driver2.leftBumper().onFalse(new InstantCommand(() -> intake.setVoltage(-7))); // USE DIFFERENT BUTTONS
        
        //m_driver2.leftTrigger().onTrue(new InstantCommand(() -> intake.setVoltage((-7))).andThen(new CoordTestingCommand(ScoringPos.ALGAEL2)));
        
        //m_driver2.leftTrigger().onFalse(new InstantCommand(() -> intake.setVoltage(0))); // USE DIFFERENT BUTTONS
        
        /* ----- Main Driver Offical Keybinds ----- */
        /* Keybinds:
         * Right Trigger = Score
         * Left Trigger = (gts?)
         * Right Bumper = High Algae
         * Left Bumper = Low Algae
         * X = Cancel All
         * A = Algae Net High
         * D-pad Up = IMU Reset
         */

        /* ----- Operator Driver Offical Keybinds ----- */
        /* Keybinds:
         * Right Trigger = Coral Intake
         * Left Trigger = Outtake
         * Right Bumper = Roll Intake-System to right side
         * Left Bumpber = Roll Intake-System to left side
         * X = L1
         * A = L2
         * B = L3
         * Y = L4
         */


        // ALGAE GROUND INTAKE - Prob not using :(
        //m_driver1.x().onTrue(new CoordTestingCommand(ScoringPos.INTAKE_ALGAE));

        // NEED BUTTON
        //m_driver1.povRight().onTrue(new CoordTestingCommand(ScoringPos.SCORE_PROCESSOR));

    }

    public static void setLiftUp(boolean up) {
        if (up) {
            MaxSpeed = LiftMaxSpeed;
            MaxAngularRate = LiftMaxAngularRate;
        } else {
            MaxSpeed = DefaultMaxSpeed;
            MaxAngularRate = DefaultMaxAngularRate;
        }
    }

    public void registeredCommands() {
        NamedCommands.registerCommand("IntakeHold", new InstantCommand(() -> intake.setVoltage(2)));
        NamedCommands.registerCommand("CoralIntake", new CoordinationCommand(ScoringPos.INTAKE_VERTICAL_CORAL).andThen(new AutoIntakeCommand()));
        NamedCommands.registerCommand("IntakeSource", new CoordinationCommand(ScoringPos.INTAKE_SOURCE).andThen(new AutoIntakeCommand()));
        NamedCommands.registerCommand("CoralL4", new InstantCommand(() -> scoreSub.setScoringLevel(4)));
        NamedCommands.registerCommand("CoralL3", new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        NamedCommands.registerCommand("CoralL1", new InstantCommand(() -> scoreSub.setScoringLevel(1)));
        NamedCommands.registerCommand("Score", new ScoringCommand());
        NamedCommands.registerCommand("GoToScore", new CoordinationCommand(ScoringPos.GO_SCORE_CORAL));
        NamedCommands.registerCommand("CoralStore", new CoordinationCommand(ScoringPos.CORAL_STORE));
        NamedCommands.registerCommand("Cancel", new InstantCommand(() -> intake.setVoltage(0))
        .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
        .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
        ));
        NamedCommands.registerCommand("HighAlgae", new CoordinationCommand(ScoringPos.ALGAEL1).andThen(new DualIntakeCommand(true)));
        NamedCommands.registerCommand("LowAlgae", new CoordinationCommand(ScoringPos.ALGAEL2).andThen(new DualIntakeCommand(true)));
        NamedCommands.registerCommand("AlgaeProcesser", new CoordinationCommand(ScoringPos.ALGAE_STORE));
        NamedCommands.registerCommand("Start", new CoordinationCommand(ScoringPos.START));
        NamedCommands.registerCommand("AlignAlgae", new AlgaeAlignCommand(drivetrain, 12));

        NamedCommands.registerCommand("StopVision", new InstantCommand(() -> CommandSwerveDrivetrain.visionOverride = true));
        NamedCommands.registerCommand("StartVision", new InstantCommand(() -> CommandSwerveDrivetrain.visionOverride = false));

    }

    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
    }
}
