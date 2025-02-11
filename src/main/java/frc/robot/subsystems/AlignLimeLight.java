package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * Does all the thinky's for where it should go... I hope
 */
public class AlignLimeLight extends SubsystemBase{

    /* ----- Swerve Drive ----- */
    private CommandSwerveDrivetrain drive;
    private SwerveDrivePoseEstimator poseEstimator;

    /* ----------- Initialization ----------- */

    public AlignLimeLight(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

        // Add it to your pose estimator
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds
        );

    }
    
}
