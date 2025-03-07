package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.config.RobotConfig;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlignPos;
import frc.robot.Constants.ScoringPos;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsytem;
/**
 * Uses April Tags to understand where it is and to align with primary april tag with certain offsets depending on which reef pole is choosen.
 */
import frc.robot.subsystems.ElevatorSubsystem;
public class AlignCommand2 extends Command {

    /* ----- April Tag ID - Positions ----- */
    private HashMap<Integer, double[]> map = new HashMap<>();

    /* ----- PIDs ----- */
    private PIDController pidX = new PIDController(3, 0, 0);
    private PIDController pidY = new PIDController(3, 0, 0);
    private PIDController pidRotate = new PIDController(4, 0, 0);

    /* ----- Subsystem Instances ----- */
    private CommandSwerveDrivetrain drive;
    private CoordinationSubsytem score = CoordinationSubsytem.getInstance();
    private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    /* ----- Variables ----- */
    private boolean hasTarget;
    private AlignPos position;
    private boolean redAlliance;
    private int tid;

    /* ----- Swerve Drive ----- */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* ----------- Initialzation ----------- */

    public AlignCommand2(CommandSwerveDrivetrain drive, AlignPos position) {
        this.position = position;
        //                X,      Y,      Rotation
        double[] tag6 = {13.474446, 3.306318, 5 * Math.PI / 3.0};
        double[] tag7 = {13.890498, 4.0259, 0};
        double[] tag8 = {13.474446, 4.745482, Math.PI / 3.0};
        double[] tag9 = {12.643358, 4.745482, 2 * Math.PI / 3.0};
        double[] tag10 = {12.227306, 4.0259, Math.PI};
        double[] tag11 = {12.643358, 3.306318, 4 * Math.PI / 3.0};
        double[] tag17 = {4.0739, 3.3063, 4 * Math.PI / 3.0};
        double[] tag18 = {3.6576, 4.0259, Math.PI};
        double[] tag19 = {4.0739, 4.7455, 2 * Math.PI / 3.0};
        double[] tag20 = {4.9047, 4.7455, Math.PI / 3.0};
        double[] tag21 = {5.3210, 4.0259, 0};
        double[] tag22 = {4.9047, 3.3063, 5 * Math.PI / 3.0};
        map.put(6, tag6);
        map.put(7, tag7);
        map.put(8, tag8);
        map.put(9, tag9);
        map.put(10, tag10);
        map.put(11, tag11);
        map.put(17, tag17);
        map.put(18, tag18);
        map.put(19, tag19);
        map.put(20, tag20);
        map.put(21, tag21);
        map.put(22, tag22);

        pidRotate.enableContinuousInput(-Math.PI, Math.PI);

        this.drive = drive;
    }

    @Override
    public void initialize() {
        redAlliance = DriverStation.getAlliance().get() == Alliance.Red;
        Pose2d currentPose = drive.getState().Pose;
        int tid = (int)LimelightHelpers.getFiducialID("limelight");
        this.tid = tid;
        if (currentPose.getX() < 2.25 && currentPose.getY() > 6) {
            hasTarget = true;
            pidX.setSetpoint(1.52);
            pidY.setSetpoint(6.05);
            pidRotate.setSetpoint(122.0 * Math.PI / 180.0);
        } else if (map.containsKey(tid)) {
            hasTarget = true;
            double[] pose = getAlignPos(map.get(tid), 0.5);
            pidX.setSetpoint(pose[0]);
            pidY.setSetpoint(pose[1]);
            pidRotate.setSetpoint(pose[2]);
        } else {
            hasTarget = false;
        }
    }

    // black red yellow green right -> left

    /* ----------- Updaters ----------- */

    /**
     * Calculates the aligned position based on the given target position.
     *
     * @param targetPos An array containing the target position with [x, y, rotation].
     * @return An array containing the aligned position with [x, y, rotation].
     */
    private double[] getAlignPos(double[] targetPos, double tagForwardOffset) {
        double tagLeftOffset = 0.173;
        if (position == AlignPos.RIGHT) {
            tagLeftOffset = -0.173;
        }
        //System.out.println(score.getPos());
        if (position == AlignPos.CENTER || score.getPos() == ScoringPos.ALGAEL1 || score.getPos() == ScoringPos.ALGAEL2) {
            tagLeftOffset = 0; // Set left offset for center
            tagForwardOffset = 0.65; ; // Set forward offset for center
        }

        if(score.getPos() == ScoringPos.GRABBED_ALGAE) {
            tagLeftOffset = 0;
            tagForwardOffset = 1.2;
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
            if (pidX.getError() < 0.02 && pidY.getError() < 0.02 && pidRotate.getError() < 0.1) {
                double[] pose = getAlignPos(map.get(tid), 0.42);
                pidX.setSetpoint(pose[0]);
                pidY.setSetpoint(pose[1]);
                pidRotate.setSetpoint(pose[2]);
            }
            // Get the current pose of the drive system
            Pose2d pose = drive.getState().Pose;

            // Calculate the power for X direction and clamp it between -1 and 1
            double powerX = pidX.calculate(pose.getX());
            double powerY = pidY.calculate(pose.getY());

            if (score.getScoringLevel() == 4) {
                powerX = MathUtil.clamp(powerX, -1, 1);
                powerY = MathUtil.clamp(powerY, -1, 1);
            } else {
                powerX = MathUtil.clamp(powerX, -2, 2);
                powerY = MathUtil.clamp(powerY, -2, 2);
            }

            powerX += .05*Math.signum(powerX);
            powerY += .05*Math.signum(powerY);
            
            Logger.recordOutput("Reefscape/Limelight/x error", pidX.getError());
            Logger.recordOutput("Reefscape/Limelight/y error", pidY.getError());


            // Calculate the rotational power and clamp it between -2 and 2
            double powerRotate = pidRotate.calculate(pose.getRotation().getRadians());
            powerRotate = MathUtil.clamp(powerRotate, -4, 4);

            if (redAlliance) {
                powerX *= -1;
                powerY *= -1;
            }

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
