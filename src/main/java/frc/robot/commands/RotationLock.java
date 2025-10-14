package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BezierCurve;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotationLock extends Command{
    
    private CommandSwerveDrivetrain drive;
    private CommandXboxController controller;

    private PIDController rotatePID = new PIDController(8, 0, 0);

    private BezierCurve driveBezier;
    private double MaxSpeed;

    private Pose2d currentPose;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RotationLock(CommandSwerveDrivetrain drive, CommandXboxController controller, BezierCurve driveBezier, double MaxSpeed) {
        this.drive = drive;
        this.controller = controller;
        this.driveBezier = driveBezier;
        this.MaxSpeed = MaxSpeed;

        rotatePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        currentPose = drive.getState().Pose;
        double setpoint = 180;
        double rotation = currentPose.getRotation().getDegrees();
        if (rotation > -45 && rotation < 45) {
            setpoint = 0;
        } else if (rotation > -135 && rotation < -45) {
            setpoint = -90;
        } else if (rotation > 45 && rotation < 135) {
            setpoint = 90;
        } else if ((rotation > 135 && rotation < 180) || (rotation > -180 && rotation < -135)) {
            setpoint = 180;
        }

        rotatePID.setSetpoint((setpoint * (Math.PI / 180)));
    }

    @Override
    public void execute() {
        currentPose = drive.getState().Pose;
        double powerRotate = rotatePID.calculate(currentPose.getRotation().getRadians());

        SwerveRequest request = driveRequest
        .withVelocityX(-driveBezier.getOutput(controller.getLeftY())  * MaxSpeed)
        .withVelocityY(-driveBezier.getOutput(controller.getLeftX()) * MaxSpeed)
        .withRotationalRate(powerRotate);

        drive.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        drive.setControl(stop);
    }
}
