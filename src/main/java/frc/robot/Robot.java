// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;

import au.grapplerobotics.CanBridge;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  //public static PowerDistribution pdh = new PowerDistribution(50, ModuleType.kRev);

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    Logger.start();

    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  @Override
  public void robotInit() {
    m_robotContainer.driveBezier.dashboardInitialSettings();
    m_robotContainer.rotateBezier.dashboardInitialSettings();
  }

  @Override
  public void disabledInit() {
    RobotContainer.toggleDrive(true);
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.driveBezier.checkAndupdateCurve();
    m_robotContainer.rotateBezier.checkAndupdateCurve();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    int[] validTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validTags);
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
