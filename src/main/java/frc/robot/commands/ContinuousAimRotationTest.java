package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BezierCurve;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ContinuousAimRotationTest extends Command {

    // double tmp = Math.atan(Y/X);

    private CommandSwerveDrivetrain drivetrain;
    private CommandXboxController controller;
    private BezierCurve bezierCurve;
    private double maxSpeed;

    private Pose2d currentFieldPose;
    private Pose2d targetFieldPose;
    private double targetDegree;

    private PIDController pidR = new PIDController(10, 0, 0);

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                                                                             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public ContinuousAimRotationTest(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, BezierCurve bezierCurve, double maxSpeed) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.bezierCurve = bezierCurve;
        this.maxSpeed = maxSpeed;

        targetFieldPose = new Pose2d(4.489332, 4.02591, new Rotation2d());

        pidR.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

        currentFieldPose = drivetrain.getState().Pose;

        targetDegree = calculateTargetDegree();

        pidR.setSetpoint(targetDegree);
    }

    @Override
    public void execute() {
        currentFieldPose = drivetrain.getState().Pose;
        targetDegree = calculateTargetDegree();

        if (currentFieldPose.getX() > 4.489332) {
            targetDegree += Math.PI;
        }

        pidR.setSetpoint(targetDegree);

        double powerR = pidR.calculate(currentFieldPose.getRotation().getRadians());

        SwerveRequest request = driveRequest
        .withVelocityX(-bezierCurve.getOutput(controller.getLeftY())  * maxSpeed)
        .withVelocityY(-bezierCurve.getOutput(controller.getLeftX()) * maxSpeed)
        .withRotationalRate(powerR);

        drivetrain.setControl(request);

        Logger.recordOutput("Reefscape/Continuous Aim/Rotation/Target Pose", targetFieldPose);
        Logger.recordOutput("Reefscape/Continuous Aim/Rotation/Target Rotation (Radians?)", targetDegree);
        Logger.recordOutput("Reefscape/Continuous Aim/Rotation/Target Rotation (Degrees?)", Units.radiansToDegrees(targetDegree));
        Logger.recordOutput("Reefscape/Continuous Aim/Rotation/Rotation Power", powerR);
        Logger.recordOutput("Reefscape/Continuous Aim/Rotation/Rotation Error", targetDegree - currentFieldPose.getRotation().getRadians());

    }

    private double calculateTargetDegree() {

        double xDiff = targetFieldPose.getX() - currentFieldPose.getX();
        double yDiff = targetFieldPose.getY() - currentFieldPose.getY();

        double degree;

        degree = Math.atan(yDiff / xDiff);

        return degree;

    }

    public void end(boolean interrupted) {
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        drivetrain.setControl(stop);
    }
    
}
