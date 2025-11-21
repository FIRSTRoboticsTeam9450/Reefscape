// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software: you can modify and/or share under the terms of the WPILib BSD license.

package frc.robot;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.FieldConstants.ReefConstants.BlueReefConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.RedReefConstants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A central place to declare constants used across the robot project.
 * This file should only contain public static final variables.
 */
public final class Constants {

  /**
   * Constants that are robot specific
   */
  public static class RobotConstants {
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

    public static class AlignConstants {
      public static final boolean runFrontLL = true;
      public static final boolean runBackLL = true;
    }

    // =========================
    // Field Alignment Offsets
    // =========================

    public static class AlignOffsets {
      public static final double leftReef = 0.173; //0.173
      public static final double rightReef = -0.173; //-0.173
      public static final double leftReefL1 = 0.2;
      public static final double rightReefL1 = -0.2;
      public static final double firstCoralBack = 0.65;
      public static final double tripleL1CoralBack = 0.7;
      public static final double scoreCoralBack = 0.44;
      public static final double scoreL3Back = 0.465;
      public static final double algaeIn = 0.65;
      public static final double algaeBack = 0.7;
      public static final double algaeLeft = 0.0;
      public static final double procOut = 1.5;
      public static final double procIn = 0.75;
    }

    public enum ClimbPos {
      STORE,
      ENGAGING,
      CLIMBING
    }

    // =========================
    // Enumerations
    // =========================

    public enum ScoringPos {
      START,
      GO_TO_SCORE,
      CORAL_STORE,
      CORAL_INTAKE_GROUND,
      CORAL_INTAKE_VERTICAL,
      CORAL_SCORE,
      CORAL_SCORE_L4,
      CORAL_PRE_L4,
      ALGAE_STORE,
      ALGAE_INTAKE_GROUND,
      ALGAE_INTAKE_REEF_LOW,
      ALGAE_INTAKE_REEF_HIGH,
      ALGAE_INTAKE_REEF_DYNAMIC,
      ALGAE_INTAKE_PROC,
      DEBUGGING_ELBOW
    }


    /** Used for aligning robot to field features */
    public enum AlignPos {
      LEFT,
      RIGHT,
      CENTER
    }

    // =========================
    // Debugging Controls
    // =========================

    public static class debugging {
      public static boolean SwerveDebugging = false;
      public static boolean LimelightDebugging = false;
      public static boolean CoordAllowedPathsDebugging = false;
      public static boolean CoordPositionDebugging = false;
      public static boolean CoordAllAtSetpoint = false;
      public static boolean ClimberPos = false;
      public static boolean CurrentPos = false;
      public static boolean DiffyTuningValues = false;
      public static boolean AlignDebugging = false;
      public static boolean IntakeDebugging = false;
    }


    // =========================
    // Standard Position Offsets
    // =========================

    public static class StatePositions {

      // double[] testArr = new double[4];
      // double[] testArr2 = {0.1, 0.2, 0.3, 0.4};

      // Pair<double[], Boolean> test = new Pair<double[],Boolean>(testArr2, false);

      /* ----------------------------------------------------- Elev -- Elbow --------------------------- DW Pitch -------------- DW Roll ------------- */
      private static final double[] CORAL_INTAKE_GROUND_ARR = {0, robotConfig.getElbowGroundPos(), robotConfig.getPitchGroundPos(), 0};
      /* ---------------------------------------------------------- Algae? -- Roll Closest Side? */
      private static final boolean[] CORAL_INTAKE_GROUND_BOOLEAN_ARR = {false, false};
      public static final Pair<double[], boolean[]> CORAL_INTAKE_GROUND_PAIR = new Pair<double[],boolean[]>(CORAL_INTAKE_GROUND_ARR, CORAL_INTAKE_GROUND_BOOLEAN_ARR);

      private static final double[] CORAL_INTAKE_VERTICAL_ARR = {0, -28, -65, 0};
      private static final boolean[] CORAL_INTAKE_VERTICAL_BOOLEAN_ARR = {false, true};
      public static final Pair<double[], boolean[]> CORAL_INTAKE_VERTICAL_PAIR = new Pair<double[],boolean[]>(CORAL_INTAKE_VERTICAL_ARR, CORAL_INTAKE_VERTICAL_BOOLEAN_ARR);

      private static final double[] ALGAE_STORE_ARR = {3, 56, -70, 0};
      private static final boolean[] ALGAE_STORE_BOOLEAN_ARR = {true, false};
      public static final Pair<double[], boolean[]> ALGAE_STORE_PAIR = new Pair<double[],boolean[]>(ALGAE_STORE_ARR, ALGAE_STORE_BOOLEAN_ARR);
        
      private static final double[] ALGAE_INTAKE_GROUND_ARR = {0, -11.68, -100.7, 0};
      private static final boolean[] ALGAE_INTAKE_GROUND_BOOLEAN_ARR = {true, false};
      public static final Pair<double[], boolean[]> ALGAE_INTAKE_GROUND_PAIR = new Pair<double[],boolean[]>(ALGAE_INTAKE_GROUND_ARR, ALGAE_INTAKE_GROUND_BOOLEAN_ARR);

      private static final double[] ALGAE_INTAKE_REEF_LOW_ARR = {11, 37.09, -110, 0};
      private static final boolean[] ALGAE_INTAKE_REEF_LOW_BOOLEAN_ARR = {true, false};
      public static final Pair<double[], boolean[]> ALGAE_INTAKE_REEF_LOW_PAIR = new Pair<double[],boolean[]>(ALGAE_INTAKE_REEF_LOW_ARR, ALGAE_INTAKE_REEF_LOW_BOOLEAN_ARR);

      private static final double[] ALGAE_INTAKE_REEF_HIGH_ARR = {20, 37.09, -110, 0};
      private static final boolean[] ALGAE_INTAKE_REEF_HIGH_BOOLEAN_ARR = {true, false};
      public static final Pair<double[], boolean[]> ALGAE_INTAKE_REEF_HIGH_PAIR = new Pair<double[],boolean[]>(ALGAE_INTAKE_REEF_HIGH_ARR, ALGAE_INTAKE_REEF_HIGH_BOOLEAN_ARR);

      private static final double[] ALGAE_INTAKE_PROC_ARR = {29, 37.09, -90, 0};
      private static final boolean[] ALGAE_INTAKE_PROC_BOOLEAN_ARR = {true, false};
      public static final Pair<double[], boolean[]> ALGAE_INTAKE_PROC_PAIR = new Pair<double[],boolean[]>(ALGAE_INTAKE_PROC_ARR, ALGAE_INTAKE_PROC_BOOLEAN_ARR);

    }
  }


  /**
   * Constants that are specific to Field
   */
  public static class FieldConstants {

    public static boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    public static final AprilTagFieldLayout FIELD_LAYOUT;

    static {
      FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
      FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public static class AprilTagIDs {
      public static final int RED_LEFT_CORAL_STATION = 1;
      public static final int RED_RIGHT_CORAL_STATION = 2;
      public static final int RED_PROCESSOR = 3;
      public static final int RED_RIGHT_NET = 4;
      public static final int RED_LEFT_NET = 5;
      public static final int RED_REEF_LEFT_DRIVER_STATION = 6;
      public static final int RED_REEF_CENTER_DRIVER_STATION = 7;
      public static final int RED_REEF_RIGHT_DRIVER_STATION = 8;
      public static final int RED_REEF_RIGHT_BARGE = 9;
      public static final int RED_REEF_CENTER_BARGE = 10;
      public static final int RED_REEF_LEFT_BARGE = 11;
      public static final int BLUE_RIGHT_CORAL_STATION = 12;
      public static final int BLUE_LEFT_CORAL_STATION = 13;
      public static final int BLUE_LEFT_BARGE = 14;
      public static final int BLUE_RIGHT_BARGE = 15;
      public static final int BLUE_PROCESSOR = 16;
      public static final int BLUE_REEF_RIGHT_DRIVER_STATION = 17;
      public static final int BLUE_REEF_CENTER_DRIVER_STATION = 18;
      public static final int BLUE_REEF_LEFT_DRIVER_STATION = 19;
      public static final int BLUE_REEF_RIGHT_BARGE = 20;
      public static final int BLUE_REEF_CENTER_BARGE = 21;
      public static final int BLUE_REEF_LEFT_BARGE = 22;
    }

    public static class LoliPopLocations {
      public static final Translation2d RED_LEFT_CORAL = new Translation2d(16.329, 2.197);
      public static final Translation2d RED_CENTER_CORAL = new Translation2d(16.329, 4.026);
      public static final Translation2d RED_RIGHT_CORAL = new Translation2d(16.329, 5.855);
      public static final Translation2d BLUE_LEFT_CORAL = new Translation2d(1.219, 5.855);
      public static final Translation2d BLUE_CENTER_CORAL = new Translation2d(1.219, 4.026);
      public static final Translation2d BLUE_RIGHT_CORAL = new Translation2d(1.219, 2.197);
    }

    public static class ReefConstants {

      public static class BlueReefConstants {
        public static final Map<Pose2d, Integer> blueAlliancePoseToTagIDsMap = Map.of(
          FieldConstants.getTag3dPose(17).toPose2d(), 17,
          FieldConstants.getTag3dPose(18).toPose2d(), 18,
          FieldConstants.getTag3dPose(19).toPose2d(), 19,
          FieldConstants.getTag3dPose(20).toPose2d(), 20,
          FieldConstants.getTag3dPose(21).toPose2d(), 21,
          FieldConstants.getTag3dPose(22).toPose2d(), 22);
      }

      public static class RedReefConstants {
        public static final Map<Pose2d, Integer> redAlliancePoseToTagIDsMap = Map.of(
          FieldConstants.getTag3dPose(6).toPose2d(), 6,
          FieldConstants.getTag3dPose(7).toPose2d(), 7,
          FieldConstants.getTag3dPose(8).toPose2d(), 8,
          FieldConstants.getTag3dPose(9).toPose2d(), 9,
          FieldConstants.getTag3dPose(10).toPose2d(), 10,
          FieldConstants.getTag3dPose(11).toPose2d(), 11);
      }

    }

    public static Pose3d getTag3dPose(int tagID) {
      if (tagID < AprilTagIDs.RED_LEFT_CORAL_STATION || tagID > AprilTagIDs.BLUE_REEF_LEFT_BARGE) {
        throw new IllegalArgumentException("Error: Given Tag ID must be between 1 and 22 (inclusive).");
      }

      return FIELD_LAYOUT.getTagPose(tagID).orElseThrow(() -> {
        final String errorMsg = String.format("getTagPose called for unexpected tag, %d", tagID);
        return new RuntimeException(errorMsg);
      });
    }
  }
}