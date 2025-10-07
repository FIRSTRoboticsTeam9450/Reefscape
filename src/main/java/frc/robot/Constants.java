// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software: you can modify and/or share under the terms of the WPILib BSD license.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotController;

/**
 * A central place to declare constants used across the robot project.
 * This file should only contain public static final variables.
 */
public final class Constants {

  // =========================
  // General Robot Configuration
  // =========================

  /** Select configuration based on the RoboRIO serial number */
  public static RobotConfig robotConfig = RobotController.getSerialNumber().equals("0329F2BF")
      ? new ThingOneConfig()
      : new ThingTwoConfig();

  /** Enables L4 scoring behavior */
  public static boolean l4mode = false;

  /** Default CAN bus used for CTRE devices */
  public static final String RIO_BUS = "Rio";
  public static final String CTRE_BUS = "CantDrive";

  /** Default neutral mode for motors */
  public static final NeutralModeValue defaultNeutral = NeutralModeValue.Brake;

  /** Read-only serial number of the RoboRIO */
  public static final String RIO_SERIAL_NUMBER = RobotController.getSerialNumber();

  // =========================
  // Controller Ports
  // =========================

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // =========================
  // Subsystem CAN IDs
  // =========================

  public static class IntakeIDs {
    public static final int kDualIntakeMotorID = 25;
    public static final int kDualIntakeCoralLaserID = 34;
    public static final int kDualIntakeAlgaeLaserID = 35;
  }

  public static class WristIDs {
    public static final int kDiffWristLeftMotorID = 23;
    public static final int kDiffWristRightMotorID = 24;
    public static final int kDiffWristPitchCANCoderID = 32;
    public static final int kDiffWristRollCANCoderID = 33;
    public static final int kElbowWristMotorID = 22;
    public static final int kElbowWristEncoderID = 31;
  }

  public static class ElevatorIDs {
    public static final int kLeftMotorID = 20;
    public static final int kRightMotorID = 21;
    public static final int kCANdiID = 30;
  }

  public static class ClimberIDs {
    public static final int kMotorID = 27;
    public static final int kEncoderID = 28;
  }

  // =========================
  // Field Alignment Offsets
  // =========================

  public static class AlignOffsets {
    public static final double leftReef = 0.173; //0.173
    public static final double rightReef = -0.173; //-0.173
    public static final double leftReefL1 = 0.224;
    public static final double rightReefL1 = -0.224;
    public static final double firstCoralBack = 0.6;
    public static final double scoreCoralBack = 0.44;
    public static final double algaeIn = 0.65;
    public static final double algaeBack = 0.7;
    public static final double algaeLeft = 0.0;
  }

  // =========================
  // Enumerations
  // =========================

  /** Describes robot's current scoring or intake position */
  public enum ScoringPos {
    START,
    INTAKE_CORAL,
    INTAKE_ALGAE,
    INTAKE_SOURCE,
    CORAL_STORE,
    ALGAE_STORE,
    SCORE_NET,
    SCORE_PROCESSOR,
    SCORE_CORAL,
    ScoreL4,
    ALGAEL1,
    ALGAEL2,
    ALGAE_COMBINED,
    GRABBED_ALGAE,
    GO_SCORE_CORAL,
    INTAKE_VERTICAL_CORAL,
    PRE_L4,
    LOLIPOP_INTAKE_ALGAE
  }

  /** Used for aligning robot to field features */
  public enum AlignPos {
    LEFT,
    RIGHT,
    CENTER
  }

  /** Reserved for future scoring tier/priority logic */
  public enum ScoringLevel {
    // Empty – define levels if needed
  }

  // =========================
  // Debugging Controls
  // =========================

  public static class debugging {
    public static final boolean SwerveDebugging = false;
    public static final boolean LimelightDebugging = false;
    public static final boolean CoordAllowedPathsDebugging = true;
    public static final boolean CoordPositionDebugging = false;
    public static final boolean CoordAllAtSetpoint = true;
    public static final boolean ClimberPos = true;
    public static final boolean currentPos = true;
    public static final boolean DiffyTuningValues = true;
  }
}