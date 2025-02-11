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

        int tid = limelight.getTid();
        if (map.containsKey(tid)) {
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

    private double[] getAlignPos(double[] targetPos) {
        double tagForwardOffset = 0.48;
        double tagLeftOffset = 0.17;
        if (position == AlignPos.RIGHT) {
            tagLeftOffset = -0.19;
        } else if (position == AlignPos.CENTER || score.getPos() == ScoringPos.ALGAEL1 || score.getPos() == ScoringPos.ALGAEL2) {
            tagLeftOffset = 0;
            tagForwardOffset = 0.75;
        }

        double rotation = targetPos[2] - Math.PI;

        if (rotation < -Math.PI) {
            rotation += 2 * Math.PI;
        } else if (rotation > Math.PI) {
            rotation -= 2 * Math.PI;
        }

        double x = targetPos[0] - tagForwardOffset * Math.cos(rotation) - tagLeftOffset * Math.sin(rotation);
        double y = targetPos[1] - tagForwardOffset * Math.sin(rotation) + tagLeftOffset * Math.cos(rotation);

        double[] out = {x, y, rotation};
        return out;
    }

    @Override
    public void execute() {
        if (hasTarget) {
            Pose2d pose = drive.getState().Pose;
            double powerX = pidX.calculate(pose.getX());
            powerX = MathUtil.clamp(powerX, -1, 1);

            double powerY = pidY.calculate(pose.getY());
            powerY = MathUtil.clamp(powerY, -1, 1);

            double powerRotate = pidRotate.calculate(pose.getRotation().getRadians());
            powerRotate = MathUtil.clamp(powerRotate, -2, 2);

            SwerveRequest request = driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
            drive.setControl(request);
        }
    }

    /* ----------- Finishers ----------- */

    @Override
    public void end(boolean interrupted) {
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        drive.setControl(stop);
    }
}
