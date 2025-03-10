// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static RobotConfig robotConfig = new ThingTwoConfig();

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class IntakeIDS {
    public static final int kDualIntakeMotorID = 25; //temp
    public static final int kDualIntakeCoralLaserID = 34;
    public static final int kDualIntakeAlgaeLaserID = 35;
  }

  public static class WristIDs {
    public static final int kDiffWristLeftMotorID = 23; //temp
    public static final int kDiffWristRightMotorID = 24; //temp
    public static final int kDiffWristPitchCANCoderID = 32; //temp
    public static final int kDiffWristRollCANCoderID = 33; //temp
    public static final int KElbowWristMotorID = 22; //temp
    public static final int KElbowWristEncoderID = 31; //temp
  }

  public static class ElevatorIDs {
    public static final int kLeftMotorID = 20;
    public static final int kRightMotorID = 21;
    public static final int kCANdiID = 30;
  }

  public static class ClimberIDs {
    public static final int kMotorID = 27;
  }

  public static class ArmConstraints {
    public static final double kRollHorizontal = 0; // temp
    public static final double kRollVerticalCoralBottom = 0; // temp
    public static final double kRollVerticalAlgaeBottom = 0; // temp
    public static final double kPitchMax = 0; // temp
    public static final double kPitchMin = 0; // temp
    public static final double kElbowMax = 0; // temp
    public static final double kElbowMin = 0; // temp
  }

  public static class ArmPositions {
    public static final double kSourceElbow = 0; // temp
    public static final double kSourcePitch = 0; // temp
    public static final double kSourceElevator = 0; // temp
    public static final double kGroundIntakeCoralElbow = 0; // temp
    public static final double kGroundIntakeCoralPitch = 0; // temp
    public static final double kGroundIntakeCoralElevator = 0; // temp
    public static final double kGroundIntakeAlgaeElbow = 0; // temp
    public static final double kGroundIntakeAlgaePitch = 0; // temp
    public static final double kGroundIntakeAlgaeElevator = 0; // temp
  }

  public static final String CTRE_BUS = "Rio";
  public static final NeutralModeValue defaultNeutral = NeutralModeValue.Brake; //normally brake, temporarly coast
  public static final String RIO_SERIAL_NUMBER = RobotController.getSerialNumber();

  public enum ScoringPos {
    START,
    INTAKE_CORAL,
    INTAKE_ALGAE,
    INTAKE_SOURCE,
    CORAL_STORE,
    ALGAE_STORE,
    SCORE_NET,
    SCORE_PROCESSOR,
    CORAL_SCOREL1,
    CORAL_SCOREL2,
    CORAL_SCOREL3,
    CORAL_SCOREL4,
    SCORE_CORAL,
    ScoreL4,
    ALGAEL1,
    ALGAEL2,
    GRABBED_ALGAE,
    GO_SCORE_CORAL,
    INTAKE_VERTICAL_CORAL
  }

  public enum AlignPos {
    LEFT,
    RIGHT,
    CENTER
  }

  public enum ScoringLevel {
    
  }

  public static class debugging {
    public static final boolean SwerveDebugging = false;
    public static final boolean LimelightDebugging = false;
    public static final boolean CoordAllowedPathsDebugging = false;
    public static final boolean CoordPositionDebugging = false;
    public static final boolean CoordAllAtSetpoint = false;
    public static final boolean ClimberPos = true;
    public static final boolean currentPos = true;
  }

}
