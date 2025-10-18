// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Watchdog;

import frc.robot.Constants.ScoringPos;
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

  //public static PowerDistribution pdh = new PowerDistribution(50, ModuleType.kRev);

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    Logger.start();

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
    dWrist.putParams(Constants.defaultNeutral);

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
    if (coordSub.getPos() != ScoringPos.START) {
      intake.setVoltage(0);
      coordSub.setPosition(ScoringPos.CORAL_STORE);
      CommandScheduler.getInstance().cancelAll();
    }
    SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.driveBezier.checkAndupdateCurve();
    m_robotContainer.rotateBezier.checkAndupdateCurve();
    dWrist.updateParams();
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
