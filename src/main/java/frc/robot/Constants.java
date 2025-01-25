// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class IntakeIDS {
    public static final int kDualIntakeMotorID = 59; //temp
    public static final int kDualIntakeCoralLaserID = 58; //temp
    public static final int kDualIntakeAlgaeLaserID = 57; //temp
  }

  public static class WristIDs {
    public static final int kDiffWristLeftMotorID = 50; //temp
    public static final int kDiffWristRightMotorID = 51; //temp
    public static final int kDiffWristPitchCANCoderID = 14; //temp
    public static final int kDiffWristRollCANCoderID = 15; //temp
    public static final int KElbowWristMotorID = 52; //temp
    public static final int KElbowWristEncoderID = 53; //temp
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
  
  public enum ScorePos {
    INTAKE_CORAL,
    INTAKE_ALGAE,
    START,
    STORE_CORAL,
    SCORE_CORAL,
    STORE_ALGAE
  }

  public enum testingPos {
    START,
    INTAKE_CORAL,
    INTAKE_ALGAE,
    INTAKE_SOURCE,
    CORAL_STORE,
    ALGAE_STORE,
    SCORE_NET,
    SCORE_PROCESSOR
  }

  public static class debugging {
    public static final boolean SwerveDebugging = false;
    public static final boolean LimelightDebugging = true;
    public static final boolean CoordAllowedPathsDebugging = true;
    public static final boolean CoordPositionDebugging = true;
  }

}
