// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Watchdog;

import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DiffWristSubsystem;
import frc.robot.subsystems.DualIntakeSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import java.lang.reflect.Field;

// import edu.wpi.first.cameraserver.CameraServer;

// import au.grapplerobotics.CanBridge;

public class Robot extends LoggedRobot {

  private static final double loopOverrunWarningTimeout = 0.2;

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private ElevatorSubsystem elev = ElevatorSubsystem.getInstance();
  private ElbowSubsystem elbow = ElbowSubsystem.getInstance();
  private DiffWristSubsystem dWrist = DiffWristSubsystem.getInstance();
  private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
  private CoordinationSubsytem coordSub = CoordinationSubsytem.getInstance();

  private final SendableChooser<Boolean> experimentalKeybindsChooser;

  private final SendableChooser<Boolean> doWeCareAboutDebuggingChooser;

  private final SendableChooser<Boolean> debuggingSwerve;
  private final SendableChooser<Boolean> debuggingLimeLight;
  private final SendableChooser<Boolean> debuggingAllowedPaths;
  private final SendableChooser<Boolean> debuggingPosition;
  private final SendableChooser<Boolean> debuggingAllAtSetpoint;
  private final SendableChooser<Boolean> debuggingClimber;
  private final SendableChooser<Boolean> debuggingCurrentPos;
  private final SendableChooser<Boolean> debuggingDiffy;
  private final SendableChooser<Boolean> debuggingAlign;
  private final SendableChooser<Boolean> debuggingIntake;

  //public static PowerDistribution pdh = new PowerDistribution(50, ModuleType.kRev);

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    Logger.start();

    experimentalKeybindsChooser = new SendableChooser<>();
    doWeCareAboutDebuggingChooser = new SendableChooser<>();
    debuggingSwerve = new SendableChooser<>();
    debuggingLimeLight = new SendableChooser<>();
    debuggingAllowedPaths = new SendableChooser<>();
    debuggingPosition = new SendableChooser<>();
    debuggingAllAtSetpoint = new SendableChooser<>();
    debuggingClimber = new SendableChooser<>();
    debuggingCurrentPos = new SendableChooser<>();
    debuggingDiffy = new SendableChooser<>();
    debuggingAlign = new SendableChooser<>();
    debuggingIntake = new SendableChooser<>();

    m_robotContainer = new RobotContainer();
    // CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  @Override
  public void robotInit() {
    m_robotContainer.driveBezier.dashboardInitialSettings();
    m_robotContainer.rotateBezier.dashboardInitialSettings();
    elev.putParams();
    elbow.putParams();
    dWrist.putParams(RobotConstants.defaultNeutral);

    experimentalKeybindsChooser.addOption("Experimental Keybinds", true);
    experimentalKeybindsChooser.setDefaultOption("Normal Keybinds", false);

    doWeCareAboutDebuggingChooser.addOption("Update Debugging", true);
    doWeCareAboutDebuggingChooser.setDefaultOption("Dont Update Debugging", false);

    debuggingSwerve.addOption("Show Swerve Data", true);
    debuggingSwerve.setDefaultOption("Hide Swerve Data", false);

    debuggingLimeLight.addOption("Show LimeLight Data", true);
    debuggingLimeLight.setDefaultOption("Hide LimeLight Data", false);

    debuggingAllowedPaths.addOption("Show Allowed Paths Data", true);
    debuggingAllowedPaths.setDefaultOption("Hide Allowed Paths Data", false);

    debuggingPosition.addOption("Show Position Data", true);
    debuggingPosition.setDefaultOption("Hide Position Data", false);

    debuggingAllAtSetpoint.addOption("Show All At Setpoint Data", true);
    debuggingAllAtSetpoint.setDefaultOption("Hide All At Setpoint Data", false);

    debuggingClimber.addOption("Show Climber Data", true);
    debuggingClimber.setDefaultOption("Hide Climber Data", false);

    debuggingCurrentPos.addOption("Show Current Position Data", true);
    debuggingCurrentPos.setDefaultOption("Hide Current Position Data", false);

    debuggingDiffy.addOption("Show Diffy Data", true);
    debuggingDiffy.setDefaultOption("Hide Diffy Data", false);

    debuggingAlign.addOption("Show Align Data", true);
    debuggingAlign.setDefaultOption("Hide Align Data", false);

    debuggingIntake.addOption("Show Intake Data", true);
    debuggingIntake.setDefaultOption("Hide Intake Data", false);

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);
  }

  @Override
  public void disabledInit() {
    RobotContainer.toggleDrive(true);
    if (coordSub.getPos() != RobotConstants.ScoringPos.START) {
      intake.setVoltage(0);
      coordSub.setPosition(RobotConstants.ScoringPos.CORAL_STORE);
      CommandScheduler.getInstance().cancelAll();
    }
    SignalLogger.stop();
    SmartDashboard.putData("Experimental Keybinds", experimentalKeybindsChooser);

    SmartDashboard.putData("Debugging", doWeCareAboutDebuggingChooser);

    SmartDashboard.putData("Debugging Swerve", debuggingSwerve);

    SmartDashboard.putData("Debugging LimeLight", debuggingLimeLight);

    SmartDashboard.putData("Debugging Allowed Paths", debuggingAllowedPaths);

    SmartDashboard.putData("Debugging Position", debuggingPosition);

    SmartDashboard.putData("Debugging All At Setpoints", debuggingAllAtSetpoint);

    SmartDashboard.putData("Debugging Climber", debuggingClimber);

    SmartDashboard.putData("Debugging Current Position", debuggingCurrentPos);

    SmartDashboard.putData("Debugging Diffy", debuggingDiffy);

    SmartDashboard.putData("Debugging Align", debuggingAlign);

    SmartDashboard.putData("Debugging Intake", debuggingIntake);
  }

  @Override
  public void disabledPeriodic() {

    m_robotContainer.driveBezier.checkAndupdateCurve();
    m_robotContainer.rotateBezier.checkAndupdateCurve();
    dWrist.updateParams();

    SmartDashboard.putBoolean("Experimental Keybinds choosen setting", experimentalKeybindsChooser.getSelected());
    SmartDashboard.putBoolean("Debugging Setting", doWeCareAboutDebuggingChooser.getSelected());
    SmartDashboard.putBoolean("Debugging Swerve Setting", debuggingSwerve.getSelected());
    SmartDashboard.putBoolean("Debugging LimeLight Setting", debuggingLimeLight.getSelected());
    SmartDashboard.putBoolean("Debugging Allowed Paths Setting", debuggingAllowedPaths.getSelected());
    SmartDashboard.putBoolean("Debugging Position Setting", debuggingPosition.getSelected());
    SmartDashboard.putBoolean("Debugging All At Setpoints Setting", debuggingAllAtSetpoint.getSelected());
    SmartDashboard.putBoolean("Debugging Climber Setting", debuggingClimber.getSelected());
    SmartDashboard.putBoolean("Debugging Current Position Setting", debuggingCurrentPos.getSelected());
    SmartDashboard.putBoolean("Debugging Diffy Setting", debuggingDiffy.getSelected());
    SmartDashboard.putBoolean("Debugging Align Setting", debuggingAlign.getSelected());
    SmartDashboard.putBoolean("Debugging Intake Setting", debuggingIntake.getSelected());



    boolean doWeCareAboutDebuggingBoolean = SmartDashboard.getBoolean("Debugging Setting", false);
    boolean swerveSetting = SmartDashboard.getBoolean("Debugging Swerve Setting", false);
    boolean limelightSetting = SmartDashboard.getBoolean("Debugging LimeLight Setting", false);
    boolean allowedPathsSetting = SmartDashboard.getBoolean("Debugging Allowed Paths Setting", false);
    boolean positionSetting = SmartDashboard.getBoolean("Debugging Position Setting", false);
    boolean allAtSetpointSetting = SmartDashboard.getBoolean("Debugging All At Setpoint Setting", false);
    boolean climberSetting = SmartDashboard.getBoolean("Debugging Climber Setting", false);
    boolean currentPositionSetting = SmartDashboard.getBoolean("Debugging Current Position Setting", false);
    boolean diffySetting = SmartDashboard.getBoolean("Debugging Diffy Setting", false);
    boolean alignSetting = SmartDashboard.getBoolean("Debugging Align Setting", false);
    boolean intakeSetting = SmartDashboard.getBoolean("Debugging Intake Setting", false);


    if (doWeCareAboutDebuggingBoolean) {
      if (RobotConstants.debugging.SwerveDebugging != swerveSetting) {
        RobotConstants.debugging.SwerveDebugging = swerveSetting;
      }
      if (RobotConstants.debugging.LimelightDebugging != limelightSetting) {
        RobotConstants.debugging.LimelightDebugging = limelightSetting;
      }
      if (RobotConstants.debugging.CoordAllowedPathsDebugging != allowedPathsSetting) {
        RobotConstants.debugging.CoordAllowedPathsDebugging = allowedPathsSetting;
      }
      if (RobotConstants.debugging.CoordPositionDebugging != positionSetting) {
        RobotConstants.debugging.CoordPositionDebugging = positionSetting;
      }
      if (RobotConstants.debugging.CoordAllAtSetpoint != allAtSetpointSetting) {
        RobotConstants.debugging.CoordAllAtSetpoint = allAtSetpointSetting;
      }
      if (RobotConstants.debugging.ClimberPos != climberSetting) {
        RobotConstants.debugging.ClimberPos = climberSetting;
      }
      if (RobotConstants.debugging.CurrentPos != currentPositionSetting) {
        RobotConstants.debugging.CurrentPos = currentPositionSetting;
      }
      if (RobotConstants.debugging.DiffyTuningValues != diffySetting) {
        RobotConstants.debugging.DiffyTuningValues = diffySetting;
      }
      if (RobotConstants.debugging.AlignDebugging != alignSetting) {
        RobotConstants.debugging.AlignDebugging = alignSetting;
      }
      if (RobotConstants.debugging.IntakeDebugging != intakeSetting) {
        RobotConstants.debugging.IntakeDebugging = intakeSetting;
      }
    }
  }

  @Override
  public void disabledExit() {
    SignalLogger.start();
  }

  @Override
  public void autonomousInit() {
    int[] validTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-coral", validTags);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-back", validTags);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    elev.updateParams();
    elbow.updateParams();

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.toggleDrive(false);
  }

  @Override
  public void testPeriodic() {
    SmartDashboard.putBoolean("Reefscape/DriverStation/ Test mode", DriverStation.isTest());
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
