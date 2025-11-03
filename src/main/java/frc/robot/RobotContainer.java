// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.InternalButton
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.robotConstants.*;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DualIntakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.ManualPitchCommand;
import frc.robot.commands.ResetIMUCommand;
import frc.robot.commands.RollSideSwitcher;
import frc.robot.commands.RotationLock;
import frc.robot.commands.ScoreOrIntakeCommand;
import frc.robot.commands.ScoringCommand;
import frc.robot.commands.ScoringCommandAuto;
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

    private final CommandXboxController m_operator = new CommandXboxController(1);
    private final CommandXboxController m_driver = new CommandXboxController(0);

    private final CommandXboxController m_EXPDriver = new CommandXboxController(4);
    private final CommandXboxController m_EXPDriverExtra = new CommandXboxController(5);

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
        configureAutos();

    }

    private void configureAutos() {
        
        autoChooser.setDefaultOption("Back Reef", drivetrain.getAutoPath("BackReef", false));
        autoChooser.addOption("Left 3 Coral", drivetrain.getAutoPath("Ground3Coral", false));
        autoChooser.addOption("Right 3 Coral", drivetrain.getAutoPath("Ground3CoralRightFr", true));
        autoChooser.addOption("Algae Steal", drivetrain.getAutoPath("Algae stealer", false));
        autoChooser.addOption("Processor", drivetrain.getAutoPath("Processor", false));


        //Back Reef Auto with hopefully the ability to detect if we have successfuly grabbed a coral, and if not go grab a diff one
        Command smartAutoTestFull = 
            Commands.sequence(

                new PathPlannerAuto("Smart Auto Start"),

                Commands.either(
                    Commands.sequence(
                        new PathPlannerAuto("Smart Auto Grabbed"),
                        Commands.either(
                            new PathPlannerAuto("Smart Auto Grabbed Grabbed"),
                            new PathPlannerAuto("Smart Auto Grabbed Missed"),
                            intake::hasCoral
                        )
                    ),
                    Commands.sequence(
                        new PathPlannerAuto("Smart Auto Missed"),
                        Commands.either(
                            new PathPlannerAuto("Smart Auto Missed Grabbed"),
                            new PathPlannerAuto("Smart Auto Missed Missed"),
                            intake::hasCoral
                        )
                    ),
                    intake::hasCoral
                )

            );
        autoChooser.addOption("Smart Auto Test Full", smartAutoTestFull);



        Command smartAutoTestSimple = 
            Commands.sequence(
                new PathPlannerAuto("Smart Auto Start"),
                Commands.either(
                    new PathPlannerAuto("Smart Auto Grabbed"), 
                    new PathPlannerAuto("Smart Auto Missed"), 
                    intake::hasCoral)
            );
        autoChooser.addOption("Smart Auto Test Simple", smartAutoTestSimple);

        Command smallSmartAuto =
            Commands.sequence(
                new PathPlannerAuto("SS Start"),
                Commands.either(
                    new PathPlannerAuto("SS Grabbed"), 
                    new PathPlannerAuto("SS Missed"), 
                    intake::hasCoral
                )
            );
        autoChooser.addOption("Small Smart Auto", smallSmartAuto);


        
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveBezier.getOutput(m_driver.getLeftY())  * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveBezier.getOutput(m_driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rotateBezier.getOutput(m_driver.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-driveBezier.getOutput(m_EXPDriver.getLeftY())  * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driveBezier.getOutput(m_EXPDriver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-rotateBezier.getOutput(m_EXPDriver.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        //Manual pitch and Elev adjustments on Operator
        // scoreSub.setDefaultCommand(new ManualPitchCommand(() -> -m_driver2.getLeftY()));
        // elevator.setDefaultCommand(new ManualElevatorCommand(() -> m_driver2.getRightY()));

        drivetrain.registerTelemetry(logger::telemeterize);










        /* ----- Main Driver Keybinds ----- */
        /* 
        * ┌──────────────┐
        * │ KEYBIND LIST │
        * └──────────────┘
        * Right Trigger     → Score
        * Left Trigger      → Go To Scoring Position / Driver Intake (Drives forward if intaking)
        * Left Bumper       → Flip (Roll Side Switch)
        * Y                 → Proccesor Algae Intake
        * Left Stick        → Movement / Align Left
        * Right Stick       → Rotation / Align Right
        * D-pad Right       → Store Climber
        * Start             → Score Pos extend toggle
        */

        // ──────────────── Keybind Command Assignments ────────────────

        // Triggers
        // m_driver.rightTrigger().onTrue(
        //     new ScoringCommand()
        // );
        m_driver.rightTrigger().onTrue(
            new ScoreOrIntakeCommand()
        );
        m_driver.leftTrigger().onTrue(
            new DriverIntakeCommand(m_driver, drivetrain)
        );

        // m_driver.leftBumper().onTrue(
        //     new RollSideSwitcher(true)
        // );
        
        m_driver.leftBumper().whileTrue(
            new AlignCommand(drivetrain, AlignPos.LEFT, m_driver)
        );

        m_driver.rightBumper().whileTrue(
            new AlignCommand(drivetrain, AlignPos.RIGHT, m_driver)
        );

        //Sticks
        m_driver.leftStick().whileTrue(
            new AlignCommand(drivetrain, AlignPos.LEFT, m_driver)
        );
        m_driver.rightStick().whileTrue(
            new AlignCommand(drivetrain, AlignPos.RIGHT, m_driver)
        );
        
        // m_driver.a().whileTrue(
        //     new AlignCommand(drivetrain, AlignPos.LEFT, m_driver)
        // );
        // m_driver.x().whileTrue(
        //     new AlignCommand(drivetrain, AlignPos.LEFT, m_driver)
        // );
        // m_driver.y().whileTrue(
        //     new AlignCommand(drivetrain, AlignPos.RIGHT, m_driver)
        // );
        // m_driver.b().whileTrue(
        //     new AlignCommand(drivetrain, AlignPos.RIGHT, m_driver)
        // );


        //Store climber
        m_driver.povRight().onTrue(
            new ClimbCommand(ClimbPos.STORE)
        );

        m_driver.y().onTrue(
            new ResetIMUCommand(drivetrain)
        );

        // m_driver.y().onTrue(new CoordinationCommand(ScoringPos.AlgaeL3).andThen(new DualIntakeCommand(true)));
                
        m_driver.start().onTrue(
            new InstantCommand(() -> scoreSub.toggleCoralInFront())
        );


        
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
        
        m_operator.rightStick().onTrue(
            new InstantCommand(() -> intake.setVoltage(0))
                .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
                .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
        );

        // === Intake & Storage Controls ===
        // Trigger coral intake and then store it
        m_operator.rightTrigger().onTrue(
            new CoordinationCommand(ScoringPos.INTAKE_CORAL)
                .andThen(new DualIntakeCommand(false))
                .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
        );

        m_operator.leftStick().onTrue(
            new CoordinationCommand(ScoringPos.INTAKE_ALGAE)
                .andThen(new DualIntakeCommand(true))
        );
        

        // === Algae Net Controls ===
        // Deactivate algae net
        m_operator.leftTrigger().onTrue(
            new InstantCommand(() -> scoreSub.setAlgaeNet(false))
        );

        // Activate algae net
        m_operator.leftBumper().onTrue(
            new InstantCommand(() -> scoreSub.setAlgaeNet(true))
        );

        m_operator.povDown().onTrue(
            new InstantCommand(() -> CoordinationSubsytem.autoGround = !CoordinationSubsytem.autoGround)
        );

        m_operator.rightBumper().onTrue(
            new RollSideSwitcher(true)
        );


        // === Scoring Level Controls ===
        m_operator.a().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(1)));
        m_operator.x().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(2)));
        m_operator.b().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        m_operator.y().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(4)));
        m_operator.start().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(0)));

        m_operator.povLeft().onTrue(
            new ClimbCommand(ClimbPos.ENGAGING) //8 degree angle going away from robot
        );
        m_operator.povRight().onTrue(
            new CoordinationCommand(ScoringPos.ALGAE_STORE)
        );

        m_operator.povUp().whileTrue(
            new RotationLock(drivetrain, m_driver, driveBezier, MaxSpeed)
        );




        /*
         * 
         *  Experimental 1 Controller Keybinds
         * 
         * 
         */


        /*
         * Left
         */

        m_EXPDriver.leftTrigger().onTrue(
            new DriverIntakeCommand(m_EXPDriver, drivetrain) // Go To Score, Drive Forward if coral intaking, or Algae Intake
        );
        m_EXPDriver.leftBumper().whileTrue(
            new AlignCommand(drivetrain, AlignPos.LEFT, m_EXPDriver)
        );
        m_EXPDriver.back().onTrue(
            new InstantCommand(() -> scoreSub.setAlgaeNet(true))
        );
        m_EXPDriver.leftStick().onTrue(
            new RollSideSwitcher(true)
        );

        /*
         * Right
         */
        m_EXPDriver.rightTrigger().onTrue(
            new ScoreOrIntakeCommand() // Score, or coral intake
        );

        m_EXPDriver.rightBumper().whileTrue(
            new AlignCommand(drivetrain, AlignPos.RIGHT, m_EXPDriver)
        );
        m_EXPDriver.start().onTrue(
            new InstantCommand(() -> scoreSub.setAlgaeNet(false))
        );
        m_EXPDriver.rightStick().onTrue(
            new InstantCommand(() -> intake.setVoltage(0))
                .andThen(new CoordinationCommand(ScoringPos.CORAL_STORE))
                .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
        );

        /*
         * Y-X-A-B Buttons
         */
        m_EXPDriver.y().onTrue(
            new CoordinationCommand(ScoringPos.AlgaeL3)
            .andThen(new DualIntakeCommand(true))
        );
        m_EXPDriver.b().onTrue(
            new InstantCommand(() -> CoordinationSubsytem.autoGround = !CoordinationSubsytem.autoGround)
        );
        m_EXPDriver.x().onTrue(
            new ClimbCommand(ClimbPos.CLIMBING)
        );
        m_EXPDriver.a().whileTrue(
            new RotationLock(drivetrain, m_driver, driveBezier, MaxSpeed)
        );

        /*
         * D-Pad
         */
        m_EXPDriver.povRight().onTrue(
            new ClimbCommand(ClimbPos.STORE)
        );

        /*
         *  Paddles
         */
        m_EXPDriverExtra.a().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(1)));
        m_EXPDriverExtra.x().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(2)));
        m_EXPDriverExtra.b().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        m_EXPDriverExtra.y().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(4)));







            

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
        NamedCommands.registerCommand("ScoreAndStay", new ScoringCommandAuto().andThen(new InstantCommand(() -> intake.setHasCoral(false))));
        NamedCommands.registerCommand("GoToScore", new CoordinationCommand(ScoringPos.GO_SCORE_CORAL));
        NamedCommands.registerCommand("CoralStore", new CoordinationCommand(ScoringPos.CORAL_STORE));

        // Algae Related
        NamedCommands.registerCommand("HighAlgae", new CoordinationCommand(ScoringPos.ALGAEL2).andThen(new DualIntakeCommand(true)));
        NamedCommands.registerCommand("LowAlgae", new CoordinationCommand(ScoringPos.ALGAEL1).andThen(new DualIntakeCommand(true)));
        NamedCommands.registerCommand("AlgaeStore", new CoordinationCommand(ScoringPos.ALGAE_STORE).andThen(new InstantCommand(() -> intake.setVoltage(12))));

        NamedCommands.registerCommand("Net", new InstantCommand(() -> scoreSub.setAlgaeNet(true)));
        NamedCommands.registerCommand("Proc", new InstantCommand(() -> scoreSub.setAlgaeNet(false)));

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
