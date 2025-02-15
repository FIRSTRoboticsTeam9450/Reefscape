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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ScoringPos;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AlignCommand2;
import frc.robot.commands.CoordTestingCommand;
import frc.robot.commands.DiffWristCommand;
import frc.robot.commands.DualIntakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.RollSideSwitcher;
import frc.robot.commands.ScoringCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordTestingSubsystem;
import frc.robot.subsystems.DiffWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.commands.CoordTestingCommand;;

public class RobotContainer {
    private static double MaxSpeed = 2.5; // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private static double LiftMaxSpeed = 1;
    private static double LiftMaxAngularRate = RotationsPerSecond.of(.3).in(RadiansPerSecond);

    private static double DefaultMaxSpeed = 2.5;
    private static double DefaultMaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // changed to .6, originaly 1.5
    
    
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

    private DiffWristSubsystem wrist = DiffWristSubsystem.getInstance();

    private ElbowSubsystem elbow = ElbowSubsystem.getInstance();

    private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    private CoordTestingSubsystem scoreSub = CoordTestingSubsystem.getInstance();

    private LimelightSubsystem limelight = new LimelightSubsystem();

    private ClimbSubsystem climb = new ClimbSubsystem();

    public RobotContainer() {
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
                drive.withVelocityX(-m_driver1.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driver1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driver1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_driver1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driver1.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driver1.getLeftY(), -m_driver1.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driver1.back().and(m_driver2.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driver1.back().and(m_driver2.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driver1.start().and(m_driver2.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driver1.start().and(m_driver2.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        drivetrain.registerTelemetry(logger::telemeterize);








        /* ------------------------- OLD DRIVER KEYBINDS ------------------------- */


        /* ----- Main Driver Keybinds ----- */
        /* Keybinds:
         * Right Trigger = Score
         * Left Trigger = Go To Scoring Pos
         * Right Bumper = High Algae
         * Left Bumper = Low Algae
         * X = Cancel All
         * A = Algae Net High
         * B = Algae Store/processor
         * D-pad Up = IMU Reset
         */
        
        // m_driver1.rightTrigger().onTrue(new ScoringCommand());
        // m_driver1.leftTrigger().onTrue(new CoordTestingCommand(ScoringPos.GO_SCORE_CORAL));
        // m_driver1.leftBumper().onTrue(new CoordTestingCommand(ScoringPos.ALGAEL1).andThen(new DualIntakeCommand(true)));
        // m_driver1.rightBumper().onTrue(new CoordTestingCommand(ScoringPos.ALGAEL2).andThen(new DualIntakeCommand(true)));
        // m_driver1.x().onTrue(
        //     new InstantCommand(() -> intake.setVoltage(0))
        //     .andThen(new CoordTestingCommand(ScoringPos.CORAL_STORE))
        //     .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
        //     ));
        // m_driver1.a().onTrue(new CoordTestingCommand(ScoringPos.SCORE_NET));
        // m_driver1.b().onTrue(new CoordTestingCommand(ScoringPos.ALGAE_STORE));
        // m_driver1.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // m_driver1.leftStick().whileTrue(new AlignCommand2(drivetrain, true));
        // m_driver1.rightStick().whileTrue(new AlignCommand2(drivetrain, false));

        // m_driver1.povRight().onTrue(new InstantCommand(() -> climb.setVoltage(6)).andThen(new CoordTestingCommand(ScoringPos.START)));
        // m_driver1.povRight().onFalse(new InstantCommand(() -> climb.setVoltage(0)));
        
        // m_driver1.povLeft().onTrue(new InstantCommand(() -> climb.setVoltage(-12)));
        // m_driver1.povLeft().onFalse(new InstantCommand(() -> climb.setVoltage(0)));



        //m_driver1.leftBumper().onTrue(new InstantCommand(() -> elevator.setSetpoint(0)));

        //m_driver1.rightBumper().onTrue(new CoordTestingCommand(ScoringPos.INTAKE_SOURCE));

        /* ----- Operator Driver Keybinds ----- */
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

        // m_driver2.rightTrigger().onTrue(new CoordTestingCommand(ScoringPos.INTAKE_CORAL).andThen(new DualIntakeCommand(false)).andThen(new CoordTestingCommand(ScoringPos.CORAL_STORE)));
        // m_driver2.leftTrigger().onTrue(new OuttakeCommand());
        // m_driver2.leftBumper().onTrue(new RollSideSwitcher());
        // m_driver2.x().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(1)));
        // m_driver2.a().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(2)));
        // m_driver2.b().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        // m_driver2.y().onTrue(new InstantCommand(() -> scoreSub.setScoringLevel(4)));

        // m_driver2.povUp().onTrue(new CoordTestingCommand(ScoringPos.CORAL_STORE));

        //m_driver2.rightBumper().onTrue(new CoordTestingCommand(ScoringPos.INTAKE_SOURCE).andThen(new DualIntakeCommand(false).andThen(new CoordTestingCommand(ScoringPos.CORAL_STORE))));
        // m_driver2.rightBumper().onTrue(new InstantCommand(() -> elevator.reset()));


        /* ------------------------------ NEW DRIVER KEYBINDS ------------------------- */

        /* ----- Driver Controls ----- */
        /*
         * Left Joystick = Movement
         * Right Joystick = Rotation
         * Left Joystick Click = Left Pole Align
         * Right Joystick Click = Right Pole ALign
         * Y = Reset Gyro
         * X = Cancel All
         * D-pad Up = Deploy Climber
         * D-pad Down = Winch Climber
         * Right Bumper = Roll Side Switcher
         * Right Trigger = Score
         * Left Trigger = Go To Score Position
         */

        m_driver1.leftStick().whileTrue(new AlignCommand2(drivetrain, AlignPos.LEFT).onFalse(new FieldCentricCommand(drivetrain, () -> m_driver1.getLeftX(), () -> m_driver1.getLeftY(), () -> m_driver1.getRightX())));
        m_driver1.rightStick().whileTrue(new AlignCommand2(drivetrain, AlignPos.RIGHT).onFalse(new FieldCentricCommand(drivetrain, () -> m_driver1.getLeftX(), () -> m_driver1.getLeftY(), () -> m_driver1.getRightX())));

        m_driver1.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        m_driver1.x().onTrue(
           new InstantCommand(() -> intake.setVoltage(0))
           .andThen(new CoordTestingCommand(ScoringPos.CORAL_STORE))
           .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
           ));

        m_driver1.povUp().onTrue(new InstantCommand(() -> climb.setVoltage(6)).andThen(new CoordTestingCommand(ScoringPos.START)));
        m_driver1.povUp().onFalse(new InstantCommand(() -> climb.setVoltage(0)));

        m_driver1.povDown().onTrue(new InstantCommand(() -> climb.setVoltage(-12)));
        m_driver1.povDown().onFalse(new InstantCommand(() -> climb.setVoltage(0)));

        m_driver1.rightBumper().onTrue(new RollSideSwitcher());

        m_driver1.rightTrigger().onTrue(new ScoringCommand());

        m_driver1.leftTrigger().onTrue(new CoordTestingCommand(ScoringPos.GO_SCORE_CORAL));

         /* ----- Operator Controls ----- */
         /*
          * Y = L4
          * B = L3
          * X = L2
          * A = L1
          * D-pad Up = Intake High Algae
          * D-pad Left = Intake Algae Low
          * D-pad Down = Intake Algae Ground
          * Right Bumper = Set Score Net
          * Left Bumper = Intake Source
          * Right Trigger = Intake Coral Ground
          * Left Trigger = Set Processor Score
          */

        m_driver2.y().onTrue( new InstantCommand(() -> scoreSub.setScoringLevel(4)));
        m_driver2.b().onTrue( new InstantCommand(() -> scoreSub.setScoringLevel(3)));
        m_driver2.x().onTrue( new InstantCommand(() -> scoreSub.setScoringLevel(2)));
        m_driver2.a().onTrue( new InstantCommand(() -> scoreSub.setScoringLevel(1)));

        m_driver2.povUp().onTrue(new CoordTestingCommand(ScoringPos.ALGAEL2).andThen(new DualIntakeCommand(true)));
        m_driver2.povLeft().onTrue(new CoordTestingCommand(ScoringPos.ALGAEL1).andThen(new DualIntakeCommand(true)));
        m_driver2.povDown().onTrue(new CoordTestingCommand(ScoringPos.INTAKE_ALGAE).andThen(new DualIntakeCommand(true)));

        m_driver2.rightBumper().onTrue(new CoordTestingCommand(ScoringPos.SCORE_NET));
        m_driver2.leftBumper().onTrue(new CoordTestingCommand(ScoringPos.INTAKE_SOURCE).andThen(new DualIntakeCommand(false)).andThen(new CoordTestingCommand(ScoringPos.CORAL_STORE)));

        m_driver2.rightTrigger().onTrue(new CoordTestingCommand(ScoringPos.SCORE_PROCESSOR));
        m_driver2.leftTrigger().onTrue(new CoordTestingCommand(ScoringPos.INTAKE_CORAL).andThen(new DualIntakeCommand(false)).andThen(new CoordTestingCommand(ScoringPos.CORAL_STORE)));

        
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
        NamedCommands.registerCommand("CoralIntake", new CoordTestingCommand(ScoringPos.INTAKE_VERTICAL_CORAL).andThen(new DualIntakeCommand(false)).andThen(new CoordTestingCommand(ScoringPos.CORAL_STORE)));
        NamedCommands.registerCommand("CoralL4", new InstantCommand(() -> scoreSub.setScoringLevel(4)));
        NamedCommands.registerCommand("CoralL1", new InstantCommand(() -> scoreSub.setScoringLevel(1)));
        NamedCommands.registerCommand("Score", new ScoringCommand());
        NamedCommands.registerCommand("GoToScore", new CoordTestingCommand(ScoringPos.GO_SCORE_CORAL));
        NamedCommands.registerCommand("CoralStore", new CoordTestingCommand(ScoringPos.CORAL_STORE));
        NamedCommands.registerCommand("Cancel", new InstantCommand(() -> intake.setVoltage(0))
        .andThen(new CoordTestingCommand(ScoringPos.CORAL_STORE))
        .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
        ));
        NamedCommands.registerCommand("HighAlgae", new CoordTestingCommand(ScoringPos.ALGAEL1).andThen(new DualIntakeCommand(true)));
        NamedCommands.registerCommand("LowAlgae", new CoordTestingCommand(ScoringPos.ALGAEL2).andThen(new DualIntakeCommand(true)));
        NamedCommands.registerCommand("AlgaeProcesser", new CoordTestingCommand(ScoringPos.ALGAE_STORE));
        NamedCommands.registerCommand("Start", new CoordTestingCommand(ScoringPos.START));
    }

    public Command getAutonomousCommand() {
       return autoChooser.getSelected();
    }
}
