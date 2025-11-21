package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElbowSubsystem;

public class ContinuousAimAngleTest extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private ElbowSubsystem elbow = ElbowSubsystem.getInstance();

    private Pose2d currentFieldPose;
    private Pose2d targetFieldPose;

    public ContinuousAimAngleTest(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        targetFieldPose = new Pose2d(4.489332, 4.02591, new Rotation2d());
    }

    @Override
    public void initialize() {
        currentFieldPose = drivetrain.getState().Pose;
    }

    @Override
    public void execute() {

        currentFieldPose = drivetrain.getState().Pose;

        double distance = calculateDistance();
        double angle = makeReasonable(distance);

        elbow.setSetpoint(angle);

        Logger.recordOutput("Reefscape/Continuous Aim/Angle/Elbow Setpoint", angle);
        Logger.recordOutput("Reefscape/Continuous Aim/Angle/Distance", distance);
        
    }

    private double calculateDistance() {
        double out;

        double x = Math.pow(targetFieldPose.getX() - currentFieldPose.getX(), 2);
        double y = Math.pow(targetFieldPose.getY() - currentFieldPose.getY(), 2);

        out = Math.sqrt(x + y);

        return out;
    }

    private double makeReasonable(double distance) {
        distance = Units.metersToInches(distance);
        distance /= 27;
        distance = Math.pow(distance, 3);

        //IK it should already be a positive no matter what, but idc
        distance = Math.abs(distance);

        double angle = 90;

        angle -= distance;

        angle = MathUtil.clamp(angle, 0, 90);
        return angle;
    }
    
}
