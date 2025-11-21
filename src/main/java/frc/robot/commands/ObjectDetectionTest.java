package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DualIntakeSubsystem;

public class ObjectDetectionTest extends Command{

    private CommandSwerveDrivetrain drivetrain;
    private DualIntakeSubsystem intakeInstance = DualIntakeSubsystem.getInstance();

    private final String LIMELIGHT_NAME = "limelight-coral";

    PIDController pidX = new PIDController(0.5, 0, 0);
    PIDController pidY = new PIDController(0.1, 0, 0);

    Timer timer = new Timer();

    NetworkTable limelightTable =  NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);

    int startingPipelineIndex;
    int wantedPipelineIndex;
    
    double tx;
    double ty;
    double ta;
    boolean tv;

    boolean txAligning;


    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
                                                                             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public ObjectDetectionTest(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {

        txAligning = true;

        pidX.setSetpoint(0);
        pidY.setSetpoint(0);

        startingPipelineIndex = 0;
        wantedPipelineIndex = 1;

        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, wantedPipelineIndex);

        tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
        tv = LimelightHelpers.getTV(LIMELIGHT_NAME);
    }

    @Override
    public void execute() {
        Logger.recordOutput("Reefscape/Object-Tracking/TX", tx);
        Logger.recordOutput("Reefscape/Object-Tracking/TY", ty);

        tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
        ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
        ta = LimelightHelpers.getTA(LIMELIGHT_NAME);


        double[] imageIn = {tx, ty};

        double[] powerArr = updatePID(imageIn);

        Logger.recordOutput("Reefscape/Object-Tracking/Drive Power X", powerArr[0]);
        Logger.recordOutput("Reefscape/Object-Tracking/Drive Power Y", powerArr[1]);
        if (txAligning && Math.abs(tx) < 0.6 && tx != 0.00) {
            txAligning = false;
        }

        SwerveRequest request = driveRequest;
        // if (txAligning) {
        //     request = driveRequest.withVelocityX(0).withVelocityY(powerArr[0]);
        // } else {
        //     request = driveRequest.withVelocityX(1).withVelocityY(0);
        // }
        request = driveRequest.withVelocityX(2).withVelocityY(powerArr[0]);

        drivetrain.setControl(request);

        Logger.recordOutput("Reefscape/Object-Tracking/Image Aligning", txAligning);
    }

    private double[] updatePID(double[] positions) {
        double powerX = pidX.calculate(positions[0]);
        powerX = MathUtil.clamp(powerX, -2, 2);

        double powerY = pidY.calculate(positions[1]);
        powerY = MathUtil.clamp(powerY, -0.5, 0.5);

        double[] out = {powerX, powerY};
        return out;
    }

    @Override
    public boolean isFinished() {
        return intakeInstance.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        limelightTable.getEntry("pipeline").setNumber(startingPipelineIndex);

        SwerveRequest.FieldCentric stop = new FieldCentric().withVelocityX(0)
                                                            .withVelocityY(0)
                                                            .withRotationalRate(0)
                                                            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        drivetrain.setControl(stop);
    }

}
