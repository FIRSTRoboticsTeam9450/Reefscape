package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.HashMap;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignPos;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Uses April Tags to understand where it is and to align with primary april tag with certain offsets depending on which reef pole is choosen.
 */
public class AlignCommand2 extends Command {

    /* ----- April Tag ID - Positions ----- */
    private HashMap<Integer, double[]> map = new HashMap<>();

    /* ----- PIDs ----- */
    private PIDController pidX = new PIDController(3, 0, 0);
    private PIDController pidY = new PIDController(3, 0, 0);
    private PIDController pidRotate = new PIDController(4, 0, 0);

    /* ----- Subsystem Instances ----- */
    private CommandSwerveDrivetrain drive;
    private LimelightSubsystem limelight = LimelightSubsystem.getInstance();
    private CoordinationSubsytem score = CoordinationSubsytem.getInstance();

    /* ----- Variables ----- */
    private boolean hasTarget;
    private AlignPos position;

    /* ----- Swerve Drive ----- */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* ----------- Initialzation ----------- */

    public AlignCommand2(CommandSwerveDrivetrain drive, AlignPos position) {
        this.position = position;
        // < 2.25, >6

        //                X,      Y,      Rotation
        double[] tag18 = {3.6576, 4.0259, Math.PI};
        double[] tag19 = {4.0739, 4.7455, 2 * Math.PI / 3.0};
        double[] tag20 = {4.9047, 4.7455, Math.PI / 3.0};
        double[] tag21 = {5.3210, 4.0259, 0};
        map.put(18, tag18);
        map.put(19, tag19);
        map.put(20, tag20);
        map.put(21, tag21);

        pidRotate.enableContinuousInput(-Math.PI, Math.PI);

        this.drive = drive;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getState().Pose;
        int tid = limelight.getTid();
        if (currentPose.getX() < 2.25 && currentPose.getY() > 6) {
            hasTarget = true;
            pidX.setSetpoint(1.52);
            pidY.setSetpoint(6.05);
            pidRotate.setSetpoint(122.0 * Math.PI / 180.0);
        } else if (map.containsKey(tid)) {
            hasTarget = true;
            double[] pose = getAlignPos(map.get(tid));
            pidX.setSetpoint(pose[0]);
            pidY.setSetpoint(pose[1]);
            pidRotate.setSetpoint(pose[2]);
        } else {
            hasTarget = false;
        }
    }

    /* ----------- Updaters ----------- */

    /**
     * Calculates the aligned position based on the given target position.
     *
     * @param targetPos An array containing the target position with [x, y, rotation].
     * @return An array containing the aligned position with [x, y, rotation].
     */
    private double[] getAlignPos(double[] targetPos) {
        double tagForwardOffset = 0.48;
        double tagLeftOffset = 0.20;
        if (position == AlignPos.RIGHT) {
            tagLeftOffset = -0.17;
        }
        System.out.println(score.getPos());
        if (position == AlignPos.CENTER || score.getPos() == ScoringPos.ALGAEL1 || score.getPos() == ScoringPos.ALGAEL2) {
            tagLeftOffset = 0.05; // Set left offset for center
            tagForwardOffset = 0.75; ; // Set forward offset for center
        }

        // Calculate rotation relative to the target position
        double rotation = targetPos[2] - Math.PI;

        // Normalize the rotation to ensure it wraps around for the shortest distance
        if (rotation < -Math.PI) {
            rotation += 2 * Math.PI;
        } else if (rotation > Math.PI) {
            rotation -= 2 * Math.PI;
        }

        // Calculate the new x and y coordinates based on the offsets and rotation
        double x = targetPos[0] - tagForwardOffset * Math.cos(rotation) - tagLeftOffset * Math.sin(rotation);
        double y = targetPos[1] - tagForwardOffset * Math.sin(rotation) + tagLeftOffset * Math.cos(rotation);

        // Create an array with the calculated x, y, and rotation values and return it
        double[] out = {x, y, rotation};
        return out;
    }

    /**
     * Executes the drive control based on the current pose and target information.
     */
    @Override
    public void execute() {
        if (hasTarget) {
            // Get the current pose of the drive system
            Pose2d pose = drive.getState().Pose;

            // Calculate the power for X direction and clamp it between -1 and 1
            double powerX = pidX.calculate(pose.getX());
            powerX = MathUtil.clamp(powerX, -1, 1);

            // Calculate the power for Y direction and clamp it between -1 and 1
            double powerY = pidY.calculate(pose.getY());
            powerY = MathUtil.clamp(powerY, -1, 1);

            // Calculate the rotational power and clamp it between -2 and 2
            double powerRotate = pidRotate.calculate(pose.getRotation().getRadians());
            powerRotate = MathUtil.clamp(powerRotate, -2, 2);

            // Create a new swerve request with the calculated velocities and rotational rate
            SwerveRequest request = driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);

            // Set the drive control with the created request
            drive.setControl(request);
        }
    }

    /* ----------- Finishers ----------- */

    /**
     * Ends the current drive control, either when the command is finished or interrupted.
     *
     * @param interrupted True if the command was interrupted, false otherwise.
     */
    @Override
    public void end(boolean interrupted) {
        // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

        // Set the drive control with the stop request to halt all movement
        drive.setControl(stop);
    }
}
