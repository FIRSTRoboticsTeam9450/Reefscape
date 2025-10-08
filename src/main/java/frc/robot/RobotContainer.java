// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

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
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FieldCentricCommand;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.ManualPitchCommand;
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
import frc.robot.subsystems.RadioSoftware;
import frc.robot.commands.CoordinationCommand;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.DriverIntakeCommand;

public class RobotContainer {
    // Top speed when lift is up
    private static double LiftMaxSpeed = 0.8;
    private static double LiftMaxAngularRate = RotationsPerSecond.of(.24).in(RadiansPerSecond);

    private static double DefaultMaxSpeed = 5.14;

    private static double DefaultMaxAngularRate = RotationsPerSecond.of(1.125).in(RadiansPerSecond); // changed to .6, originaly 1.5
    
    // Current max speed - dont change this one
    public static double MaxSpeed = DefaultMaxSpeed;
    public static double MaxAngularRate = DefaultMaxAngularRate;

    public static double SprintSpeed = 5.14;

    public static boolean sprint = false;

    private static boolean driveEnabled = true;


    /*
     * Old Rotate Curve Values:
     * x1: 89.4
     * y1 = 0.117
     * x2: 88.5
     * y2: 0.896
     * 
     * Old Drive Curve Values:
     * x1: 89.4
     * y1: 0.117
     * x2: 88.5
     * y2: 0.896
     * 
     * Woojin Drive Curve Vals:
     * x1: 117.4
     * y1: 0.054
     * x2:91.4
     * y2:0.76
     * 
     * Woojin Rotate Curve Vals:
     * 104
     * -0.022
     * 88.5
     * 0.896
     */
    public BezierCurve driveBezier = new BezierCurve("drive", 117.4, 0.054, 91.4, 0.76, 0.07, 0.01); //deadbang original:0.07, minOutput: 0.03
    public BezierCurve rotateBezier = new BezierCurve("drive", 120.1, 0.145, 92.2, 0.362, 0.035, 0.03);
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withRotationalDeadband(MaxSpeed * 0.15)    
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driver2 = new CommandXboxController(1);

    private final CommandXboxController m_driver1 = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    private CoordinationSubsytem scoreSub = CoordinationSubsytem.getInstance();

    private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    private ClimbSubsystem climber = ClimbSubsystem.getInstance();

    private RadioSoftware radio = RadioSoftware.getInstance();
    public static double pigeonOffset = 0;

    public RobotContainer() {
        configureBindings();
        registeredCommands();

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("BackReef", drivetrain.getAutoPath("BackReef", false));
        autoChooser.addOption("Left 3 Coral", drivetrain.getAutoPath("Ground3Coral", false));
        autoChooser.addOption("Right 3 Coral", drivetrain.getAutoPath("Ground3CoralRightFr", true));
        autoChooser.addOption("Back Reef", drivetrain.getAutoPath("BackReef", false));
        autoChooser.addOption("BackReefReorderedTest", drivetrain.getAutoPath("BackReefReorderedTest", false));
        //autoChooser.addOption("Left Source", drivetrain.getAutoPath("Source", false));
        autoChooser.addOption("Left Source", drivetrain.getAutoPath("SourceAlternate", false));
        autoChooser.addOption("TEST", drivetrain.getAutoPath("TEST", false));
        autoChooser.addOption("Algae Steal", drivetrain.getAutoPath("Algae stealer", false));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
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

        //Manual pitch and Elev adjustments on Operator
        // scoreSub.setDefaultCommand(new ManualPitchCommand(() -> -m_driver2.getLeftY()));
        // elevator.setDefaultCommand(new ManualElevatorCommand(() -> m_driver2.getRightY()));

        // m_driver1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // m_driver1.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-m_driver1.getLeftY(), -m_driver1.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driver1.back().and(m_driver2.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // m_driver1.back().and(m_driver2.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driver1.start().and(m_driver2.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driver1.start().and(m_driver2.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        drivetrain.registerTelemetry(logger::telemeterize);












        /* ----- Main Driver Keybinds ----- */
        /* 
        * ┌──────────────┐
        * │ KEYBIND LIST │
        * └──────────────┘
        * Right Trigger     → Score
        * Left Trigger      → Go To Scoring Position
        * Left Bumper       → Flip (Roll Side Switch)
        * Y Button          → IMU Reset
        * Left Stick        → Movement / Align Left
        * Right Stick       → Rotation / Align Right
        * D-pad Right       → Store Climber
        * D-pad Left        → Toggle Field Centric Drive
        */

        // ──────────────── Keybind Command Assignments ────────────────

        // Triggers
        m_driver1.rightTrigger().onTrue(
            new ScoringCommand()
        );
        m_driver1.leftTrigger().onTrue(
            new DriverIntakeCommand(m_driver1, drivetrain)
        );

        // Bumpers PUT THIS BACK LATER
        m_driver1.leftBumper().onTrue(
            new RollSideSwitcher(true)
        );
        
        m_driver1.y().onTrue(
            new ResetIMUCommand(drivetrain)
        );

        // Sticks
        m_driver1.leftStick().whileTrue(
            new AlignCommand(drivetrain, AlignPos.LEFT, m_driver1)
        );
        m_driver1.rightStick().whileTrue(
            new AlignCommand(drivetrain, AlignPos.RIGHT, m_driver1)
        );


        //Store climber
        m_driver1.povRight().onTrue(
            new ClimbCommand(0.712, 4) //12
        );

        // Honestly dont know
        m_driver1.povLeft().toggleOnTrue(
            new FieldCentricCommand(
                drivetrain,
                () -> -driveBezier.getOutput(m_driver1.getLeftX()),
                () -> -driveBezier.getOutput(m_driver1.getLeftY()),
                () -> rotateBezier.getOutput(m_driver1.getRightX())
            )
        );

        m_driver1.a().whileTrue(new DriveForwardCommand(drivetrain, m_driver1));
                
        m_driver1.start().onTrue(
            new InstantCommand(() -> scoreSub.toggleCoralInFront())
        );

        /* ----- Disabled Keybinds ----- */

        // m_driver1.rightBumper().onTrue(
        //     new InstantCommand(() -> CoordinationSubsytem.autoGround = !CoordinationSubsytem.autoGround)
        // );

        // Face Buttons
        // m_driver1.x().onTrue(
        //     new InstantCommand(() -> intake.setVoltage(0))
        //         .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
        //         .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
        // );

        // m_driver1.povUp().onTrue(
        //     new ClimbCommand(0.881, 9)
        //         .andThen(new CoordinationCommand(ScoringPos.START))
        // );
        // m_driver1.povDown().onTrue(
        //     new ClimbCommand(0.20, 10.5)
        // );

        // m_driver1.povUp().whileTrue(
        //     new InstantCommand(() -> climber.setVoltage(-1))
        // );
        // m_driver1.povUp().onFalse(
        //     new InstantCommand(() -> climber.setVoltage(0))
        // );

        // m_driver1.povDown().whileTrue(
        //     new InstantCommand(() -> climber.setVoltage(1))
        // );
        // m_driver1.povDown().onFalse(
        //     new InstantCommand(() -> climber.setVoltage(0))
        // );
















        /* ----- Operator Driver Keybinds ----- */
        /*
        * ┌────────────────────┐
        * │ DRIVER 2 KEYBINDS  │
        * └────────────────────┘
        * Right Trigger     → Intake Coral Ground
        * Left Trigger      → Intake Ground Algae
        * Right Bumper      → Enable/Disable Auto Ground Intake
        * Left Bumper       → Set Net Score (Algae Net On)
        * X Button          → Set Scoring Level: L2
        * A Button          → Set Scoring Level: L1
        * B Button          → Set Scoring Level: L3
        * Y Button          → Set Scoring Level: L4
        * D-pad Left        → Climber engage
        * D-pad Down        → Intake Algae Ground
        * D-pad Right       → Climber climb
        */
        
        m_driver2.rightStick().onTrue(
            new InstantCommand(() -> intake.setVoltage(0))
                .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
                .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
        );

        // === Intake & Storage Controls ===
        // Trigger coral intake and then store it
        m_driver2.rightTrigger().onTrue(
            new CoordinationCommand(ScoringPos.INTAKE_CORAL)
                .andThen(new DualIntakeCommand(false))
                .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
        );

        // Trigger algae intake sequence
        m_driver2.povDown().onTrue(
            new CoordinationCommand(ScoringPos.INTAKE_ALGAE)
                .andThen(new DualIntakeCommand(true))
        );

        m_driver2.leftStick().onTrue(
            new CoordinationCommand(ScoringPos.INTAKE_ALGAE)
                .andThen(new DualIntakeCommand(true))
        );
        

        // === Algae Net Controls ===
        // Deactivate algae net
        m_driver2.leftTrigger().onTrue(
            new InstantCommand(() -> scoreSub.setAlgaeNet(false))
        );

        // Activate algae net
        m_driver2.leftBumper().onTrue(
            new InstantCommand(() -> scoreSub.setAlgaeNet(true))
        );

        m_driver2.rightBumper().onTrue(
            new InstantCommand(() -> CoordinationSubsytem.autoGround = !CoordinationSubsytem.autoGround)
        );


        // === Scoring Level Controls ===
        m_driver2.a().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(1)));
        m_driver2.x().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(2)));
        m_driver2.b().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        m_driver2.y().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(4)));

        m_driver2.povLeft().onTrue(
            new ClimbCommand(0.14, 12) //8 degree angle going away from robot
                .andThen(new CoordinationCommand(ScoringPos.START))
        );
        m_driver2.povRight().onTrue(
            new ClimbCommand(0.634, 8) //10.5
        );

        /* ----- Disabled Keybinds ----- */

        // m_driver2.povRight().onTrue(
        //     new CoordinationCommand(ScoringPos.ALGAE_COMBINED)
        //         // .andThen(new DualIntakeCommand(true))
        // );

        // // === Algae Positioning Controls ===
        // // Score algae at level 1 or 2 depending on POV
        // m_driver2.povLeft().onTrue(
        //     new CoordinationCommand(ScoringPos.ALGAEL1)
        //         .andThen(new DualIntakeCommand(true))
        // );
        // m_driver2.povUp().onTrue(
        //     new CoordinationCommand(ScoringPos.ALGAEL2)
        //         .andThen(new DualIntakeCommand(true))
        // );












        // === Miscellaneous ===
        // Play music on command
        // m_driver2.rightBumper().onTrue(
        //     new InstantCommand(() -> radio.playMusic())
        // );

        // Optional: Uncomment if RollSideSwitcher is needed
        // m_driver2.rightBumper().onTrue(new RollSideSwitcher());

        /* ----------- Manual Tuning Assistance ----------- */
        
        // m_driver2.y().onTrue(new ElbowCommand(8.78));
        // m_driver2.x().onTrue(new ElbowCommand(26));
        // m_driver2.b().onTrue(new ElbowCommand(50));
        // m_driver2.a().onTrue(new ElbowCommand(89));

        // m_driver2.povUp().onTrue(new ElevatorCommand(38));
        // m_driver2.povRight().onTrue(new ElevatorCommand(26));
        // m_driver2.povLeft().onTrue(new ElevatorCommand(13));
        // m_driver2.povDown().onTrue(new ElevatorCommand(0));

        // m_driver2.y().onTrue(new DiffWristCommand(0, -60));
        // m_driver2.x().onTrue(new DiffWristCommand(90, -90));
        // m_driver2.b().onTrue(new DiffWristCommand(-90, -90));
        // m_driver2.a().onTrue(new DiffWristCommand(0, -120));

        /* ----- Commands not currently in use ----- */
        
        // SOURCE INTAKE
        //  m_driver2.rightBumper().onTrue(new CoordinationCommand(ScoringPos.INTAKE_SOURCE).andThen(new DualIntakeCommand(false).andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))));
    
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
        // Intake Commands
        NamedCommands.registerCommand("IntakeHold", new InstantCommand(() -> intake.setVoltage(2)));
        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> intake.setVoltage(-2)));
        NamedCommands.registerCommand("IntakeStop", new InstantCommand(() -> intake.setVoltage(0)));
        NamedCommands.registerCommand("CoralIntake", new CoordinationCommand(ScoringPos.INTAKE_VERTICAL_CORAL).andThen(new AutoIntakeCommand(false)));
        NamedCommands.registerCommand("IntakeSource", new CoordinationCommand(ScoringPos.INTAKE_SOURCE).andThen(new AutoIntakeCommand(true)));

        NamedCommands.registerCommand("FlipIntake", new RollSideSwitcher(true));

        // Scoring Level Selection
        NamedCommands.registerCommand("CoralL4", new InstantCommand(() -> scoreSub.setScoringLevel(4)));
        NamedCommands.registerCommand("CoralL3", new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        NamedCommands.registerCommand("CoralL2", new InstantCommand(() -> scoreSub.setScoringLevel(2)));
        NamedCommands.registerCommand("CoralL1", new InstantCommand(() -> scoreSub.setScoringLevel(1)));

        // Scoring Actions
        NamedCommands.registerCommand("Score", new ScoringCommand().andThen(new InstantCommand(() -> intake.setHasCoral(false))));
        NamedCommands.registerCommand("GoToScore", new CoordinationCommand(ScoringPos.GO_SCORE_CORAL));
        NamedCommands.registerCommand("CoralStore", new CoordinationCommand(ScoringPos.CORAL_STORE));

        // Algae Related
        NamedCommands.registerCommand("HighAlgae", new CoordinationCommand(ScoringPos.ALGAEL2).andThen(new DualIntakeCommand(true)));
        NamedCommands.registerCommand("LowAlgae", new CoordinationCommand(ScoringPos.ALGAEL1).andThen(new DualIntakeCommand(true)));
        NamedCommands.registerCommand("AlgaeStore", new CoordinationCommand(ScoringPos.ALGAE_STORE).andThen(new InstantCommand(() -> intake.setVoltage(12))));
        NamedCommands.registerCommand("AlignAlgae", new AlgaeAlignCommand(drivetrain, -18));

        // Start & Cancel Routines
        NamedCommands.registerCommand("Start", new CoordinationCommand(ScoringPos.START));
        NamedCommands.registerCommand("Cancel", new InstantCommand(() -> intake.setVoltage(0))
            .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
            .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
        );

        // Vision Control
        NamedCommands.registerCommand("StopVision", new InstantCommand(() -> CommandSwerveDrivetrain.completeVisionOverride = true));
        NamedCommands.registerCommand("StartVision", new InstantCommand(() -> CommandSwerveDrivetrain.completeVisionOverride = false));

        NamedCommands.registerCommand("StopFrontVision", new InstantCommand(() -> CommandSwerveDrivetrain.frontVisionOverride = true));
        NamedCommands.registerCommand("StartFrontVision", new InstantCommand(() -> CommandSwerveDrivetrain.frontVisionOverride = false));

        NamedCommands.registerCommand("StopBackVision", new InstantCommand(() -> CommandSwerveDrivetrain.backVisionOverride = true));
        NamedCommands.registerCommand("StartBackVision", new InstantCommand(() -> CommandSwerveDrivetrain.backVisionOverride = false));

        // Wait Logic
        NamedCommands.registerCommand("WaitForCoral", new WaitForLaserCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
