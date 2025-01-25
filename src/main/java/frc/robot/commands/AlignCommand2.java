package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignCommand2 extends Command {

    PIDController pidX = new PIDController(3, 0, 0);
    PIDController pidY = new PIDController(3, 0, 0);
    PIDController pidRotate = new PIDController(4, 0, 0);

    CommandSwerveDrivetrain drive;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AlignCommand2(CommandSwerveDrivetrain drive) {
        pidX.setSetpoint(3.13);
        pidY.setSetpoint(4.24);
        pidRotate.setSetpoint(0);

        this.drive = drive;
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getState().Pose;
        double powerX = pidX.calculate(pose.getX());
        powerX = MathUtil.clamp(powerX, -0.5, 0.5);

        double powerY = pidY.calculate(pose.getY());
        powerY = MathUtil.clamp(powerY, -0.5, 0.5);

        double powerRotate = pidRotate.calculate(pose.getRotation().getRadians());
        powerRotate = MathUtil.clamp(powerRotate, -1, 1);

        SwerveRequest request = driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
        drive.setControl(request);
    }

    




    
}
