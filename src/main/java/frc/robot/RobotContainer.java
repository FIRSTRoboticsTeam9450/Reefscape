// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlignPos;
import frc.robot.Constants.ScoringPos;
import frc.robot.commands.AlgaeAlignCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DualIntakeCommand;
import frc.robot.commands.FieldCentricCommand;
import frc.robot.commands.GoToScorePosCommand;
import frc.robot.commands.ManualPitchCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ResetIMUCommand;
import frc.robot.commands.RollSideSwitcher;
import frc.robot.commands.ScoringCommand;
import frc.robot.commands.WaitForLaserCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.CoordinationCommand;
import frc.robot.commands.DriverIntakeCommand;

public class RobotContainer {
    // Top speed when lift is up
    private static double LiftMaxSpeed = 1;
    private static double LiftMaxAngularRate = RotationsPerSecond.of(.3).in(RadiansPerSecond);

    // Normal top speed
    private static double DefaultMaxSpeed = 4.369;
    private static double DefaultMaxAngularRate = RotationsPerSecond.of(1.125).in(RadiansPerSecond); // changed to .6, originaly 1.5
    
    // Current max speed - dont change this one
    public static double MaxSpeed = DefaultMaxSpeed;
    public static double MaxAngularRate = DefaultMaxAngularRate;

    public static double SprintSpeed = 5.14;

    public static boolean sprint = false;

    private static boolean driveEnabled = true;

    public BezierCurve driveBezier = new BezierCurve("drive", 89.4, 0.117, 88.5, 0.896, 0.07, 0.03);
    public BezierCurve rotateBezier = new BezierCurve("drive", 89.4, 0.117, 88.5, 0.896, 0.07, 0.03);
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withRotationalDeadband(MaxSpeed * 0.15)    
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

    private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    private ClimbSubsystem climb = ClimbSubsystem.getInstance();

    public static double pigeonOffset = 0;

    public RobotContainer() {
        configureBindings();
        registeredCommands();

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Left 3 Coral", drivetrain.getAutoPath("Ground3Coral", false));
        autoChooser.addOption("Right 3 Coral", drivetrain.getAutoPath("Ground3CoralRightFr", true));
        autoChooser.addOption("Back Reef", drivetrain.getAutoPath("BackReef", false));
        //autoChooser.addOption("Left Source", drivetrain.getAutoPath("Source", false));
        autoChooser.addOption("Left Source", drivetrain.getAutoPath("SourceAlternate", false));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        elevator.setController(m_driver1);
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveBezier.getOutput(m_driver1.getLeftY())  * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveBezier.getOutput(m_driver1.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rotateBezier.getOutput(m_driver1.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
         * Right Trigger = Score
         * Left Trigger = Go To Scoring Pos
         * Right Bumper = Outtake
         * Left Bumper = Flip
         * X = Cancel All + Store
         * Y = IMU Reset
         * Left Stick = Movement/Align Left
         * Right Stick = Rotate/Align Right
         * D-pad Up = Deploy Climber
         * D-pad Down = Climb
         * D-pad Right = Store Climber
         */
        
        m_driver1.rightTrigger().onTrue(new ScoringCommand());
        m_driver1.leftTrigger().onTrue(new DriverIntakeCommand(m_driver1, drivetrain));
        m_driver1.leftBumper().onTrue(new RollSideSwitcher(true));
        m_driver1.rightBumper().onTrue(new InstantCommand(() -> CoordinationSubsytem.autoGround = !CoordinationSubsytem.autoGround));
        m_driver1.x().onTrue(
            new InstantCommand(() -> intake.setVoltage(0))
            .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
            .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
            ));
        m_driver1.y().onTrue(new ResetIMUCommand(drivetrain));
        m_driver1.leftStick().whileTrue(new AlignCommand(drivetrain, AlignPos.LEFT, m_driver1));
        m_driver1.rightStick().whileTrue(new AlignCommand(drivetrain, AlignPos.RIGHT, m_driver1));
        m_driver1.povUp().onTrue(new ClimbCommand(0.9, 12));
        m_driver1.povDown().onTrue(new ClimbCommand(0.300, 9).andThen(new CoordinationCommand(ScoringPos.START)));
        m_driver1.povRight().onTrue(new ClimbCommand(0.1, 3));
        
        m_driver1.povLeft().toggleOnTrue(new FieldCentricCommand(drivetrain, () -> -driveBezier.getOutput(m_driver1.getLeftX()), 
            () -> -driveBezier.getOutput(m_driver1.getLeftY()), 
            () -> rotateBezier.getOutput(m_driver1.getRightX())));
        
        m_driver1.start().onTrue(new InstantCommand(() -> scoreSub.toggleCoralInFront()));

        /* ----- Operator Driver Keybinds ----- */
        /* Keybinds:
         * Right Trigger = Intake Coral Ground
         * Left Trigger = Set Processor Score
         * Right Bumper = Flip
         * Left Bumper = Set Net Score
         * X = L2
         * A = L1
         * B = L3
         * Y = L4
         * D-pad Up = Intake Algae High
         * D-pad Left = Intake Algae Low
         * D-pad Down = Intake Algae Ground
         */

        m_driver2.rightTrigger().onTrue(new CoordinationCommand(ScoringPos.INTAKE_CORAL).andThen(new DualIntakeCommand(false)).andThen(new CoordinationCommand(ScoringPos.CORAL_STORE)));
        m_driver2.leftTrigger().onTrue(new InstantCommand(() -> scoreSub.setAlgaeNet(false)));
        m_driver2.leftBumper().onTrue(new InstantCommand(() -> scoreSub.setAlgaeNet(true)));
        //m_driver2.rightBumper().onTrue(new RollSideSwitcher());
        m_driver2.x().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(2)));
        m_driver2.a().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(1)));
        m_driver2.b().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        m_driver2.y().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(4)));
        
        m_driver2.povUp().onTrue(new CoordinationCommand(ScoringPos.ALGAEL2).andThen(new DualIntakeCommand(true)));
        m_driver2.povLeft().onTrue(new CoordinationCommand(ScoringPos.ALGAEL1).andThen(new DualIntakeCommand(true)));
        m_driver2.povDown().onTrue(new CoordinationCommand(ScoringPos.INTAKE_ALGAE).andThen(new DualIntakeCommand(true)));
        m_driver2.povRight().onTrue(new CoordinationCommand(ScoringPos.LOLIPOP_INTAKE_ALGAE).andThen(new DualIntakeCommand(true)));

        m_driver2.rightStick().onTrue(new ClimbCommand(0.91, 12));
        m_driver2.leftStick().onTrue(new OuttakeCommand());

        /* ----- Commands not currently in use ----- */
        
        // SOURCE INTAKE
        m_driver2.rightBumper().onTrue(new CoordinationCommand(ScoringPos.INTAKE_SOURCE).andThen(new DualIntakeCommand(false).andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))));
    
        // VERTICAL CORAL
        //m_driver2.rightStick().onTrue(new CoordinationCommand(ScoringPos.INTAKE_VERTICAL_CORAL).andThen(new DualIntakeCommand(false)));
        
        // UNCOMMENT FOR MANUAL CLIMB
        //m_driver1.povRight().onTrue(new InstantCommand(() -> climb.setVoltage(4))).onFalse(new InstantCommand(() -> climb.setVoltage(0)));
        //m_driver1.povLeft().onTrue(new InstantCommand(() -> climb.setVoltage(-4))).onFalse(new InstantCommand(() -> climb.setVoltage(0)));
    }

    public static void setLiftUp(boolean up) {
        if (!driveEnabled) {
            return;
        }
        if (up) {
            MaxSpeed = LiftMaxSpeed;
            MaxAngularRate = LiftMaxAngularRate;
        } else {
            if (sprint) {
                MaxSpeed = SprintSpeed;
            } else {
                MaxSpeed = DefaultMaxSpeed;
            }
            MaxAngularRate = DefaultMaxAngularRate;
        }
    }

    public static void toggleDrive(boolean enabled) {
        if (enabled) {
            driveEnabled = true;
            MaxSpeed = DefaultMaxSpeed;
            MaxAngularRate = DefaultMaxAngularRate;
        } else {
            driveEnabled = false;
            MaxSpeed = 0;
            MaxAngularRate = 0;
        }
    }

    public void registeredCommands() {
        NamedCommands.registerCommand("IntakeHold", new InstantCommand(() -> intake.setVoltage(2)));
        NamedCommands.registerCommand("CoralIntake", new CoordinationCommand(ScoringPos.INTAKE_VERTICAL_CORAL).andThen(new AutoIntakeCommand(false)));
        NamedCommands.registerCommand("IntakeSource", new CoordinationCommand(ScoringPos.INTAKE_SOURCE).andThen(new AutoIntakeCommand(true)));
        NamedCommands.registerCommand("CoralL4", new InstantCommand(() -> scoreSub.setScoringLevel(4)));
        NamedCommands.registerCommand("CoralL3", new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        NamedCommands.registerCommand("CoralL1", new InstantCommand(() -> scoreSub.setScoringLevel(1)));
        NamedCommands.registerCommand("Score", new ScoringCommand().andThen(new InstantCommand(() -> intake.setHasCoral(false))));
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
        NamedCommands.registerCommand("AlignAlgae", new AlgaeAlignCommand(drivetrain, -18));

        NamedCommands.registerCommand("StopVision", new InstantCommand(() -> CommandSwerveDrivetrain.visionOverride = true));
        NamedCommands.registerCommand("StartVision", new InstantCommand(() -> CommandSwerveDrivetrain.visionOverride = false));
        NamedCommands.registerCommand("WaitForCoral", new WaitForLaserCommand());
    }

    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
    }
}
