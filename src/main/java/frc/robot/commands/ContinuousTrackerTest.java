// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DiffWristSubsystem;
import frc.robot.subsystems.DualIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ContinuousTrackerTest extends Command {

  private HashMap<Integer, double[]> aprilTagLocationMap = new HashMap<>();

  private CommandSwerveDrivetrain drivetrain;
  private DiffWristSubsystem diffWrist = DiffWristSubsystem.getInstance();
  private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

  PIDController rotatePID = new PIDController(5, 0, 0);
  CommandXboxController controller; 
  private double tid;

  /** Creates a new ContinuousTrackerTest. */
  public ContinuousTrackerTest(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    /* --------------- AprilTag location map --------------- */
    //                X,         Y,        Rotation
    double[] tag3 = {11.560833, 8.055626, 3 * Math.PI / 2};
    double[] tag6 =  {13.474446, 3.306318, 5 * Math.PI / 3.0};
    double[] tag7 =  {13.890498, 4.0259,   0};
    double[] tag8 =  {13.474446, 4.745482, Math.PI / 3.0};
    double[] tag9 =  {12.643358, 4.745482, 2 * Math.PI / 3.0};
    double[] tag10 = {12.227306, 4.0259,   Math.PI};
    double[] tag11 = {12.643358, 3.306318, 4 * Math.PI / 3.0};
    double[] tag16 = {5.987553, -0.003810, Math.PI / 2};
    double[] tag17 = {4.0739,    3.3063,   4 * Math.PI / 3.0};
    double[] tag18 = {3.6576,    4.0259,   Math.PI};
    double[] tag19 = {4.0739,    4.7455,   2 * Math.PI / 3.0};
    double[] tag20 = {4.9047,    4.7455,   Math.PI / 3.0};
    double[] tag21 = {5.3210,    4.0259,   0};
    double[] tag22 = {4.9047,    3.3063,   5 * Math.PI / 3.0};
    aprilTagLocationMap.put(3, tag3);
    aprilTagLocationMap.put(6, tag6);
    aprilTagLocationMap.put(7, tag7);
    aprilTagLocationMap.put(8, tag8);
    aprilTagLocationMap.put(9, tag9);
    aprilTagLocationMap.put(10, tag10);
    aprilTagLocationMap.put(11, tag11);
    aprilTagLocationMap.put(16, tag16);
    aprilTagLocationMap.put(17, tag17);
    aprilTagLocationMap.put(18, tag18);
    aprilTagLocationMap.put(19, tag19);
    aprilTagLocationMap.put(20, tag20);
    aprilTagLocationMap.put(21, tag21);
    aprilTagLocationMap.put(22, tag22);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tid = LimelightHelpers.getFiducialID("limelight-coral");
    rotatePID.setSetpoint(aprilTagLocationMap.get(tid)[2]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    if(Math.abs(currentPose.getRotation().getRadians() - rotatePID.getSetpoint()) < 0.2) {
      controller.setRumble(RumbleType.kBothRumble, 1);
    }
    double rotatePower = MathUtil.clamp(rotatePID.calculate(currentPose.getRotation().getRadians()), -6, 6);
    System.out.println(rotatePower + " POWER");
    SwerveRequest request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withRotationalRate(rotatePower);
    drivetrain.setControl(request);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveRequest stop = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withRotationalRate(0);
    drivetrain.setControl(stop);
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
