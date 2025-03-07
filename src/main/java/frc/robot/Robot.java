// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static PowerDistribution pdh = new PowerDistribution(50, ModuleType.kRev);

  double x1;
  double y1;
  double x2;
  double y2;

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    Logger.start();

    m_robotContainer = new RobotContainer();
    CanBridge.runTCP();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Reefscape/DriveCurve/x1", 92.7);
    SmartDashboard.putNumber("Reefscape/DriveCurve/x2", 90);
    SmartDashboard.putNumber("Reefscape/DriveCurve/y1", 0.085);
    SmartDashboard.putNumber("Reefscape/DriveCurve/y2", 0.905);
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    x1 = SmartDashboard.getNumber("Reefscape/DriveCurve/x1", 0);
    x2 = SmartDashboard.getNumber("Reefscape/DriveCurve/x2", 0);
    y1 = SmartDashboard.getNumber("Reefscape/DriveCurve/y1", 0);
    y2 = SmartDashboard.getNumber("Reefscape/DriveCurve/y2", 0);
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
    CurveTest.generateCurve(x1, y1, x2, y2);
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
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
