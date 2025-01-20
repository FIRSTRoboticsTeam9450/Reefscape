package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    //Limelight Data table
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
    //Limelight specific datas
    NetworkTableEntry tx = limelight.getEntry("tx");
    NetworkTableEntry ty = limelight.getEntry("ty");
    NetworkTableEntry ta = limelight.getEntry("ta");
    NetworkTableEntry botPose = limelight.getEntry("botpose_targetspace");
    NetworkTableEntry botPoseField = limelight.getEntry("botpose_wpiblue");
    public static LimelightSubsystem LL; 

    /* ----- Updaters ----- */

    @Override
    public void periodic() {
        SmartDashboard.putNumber("limelight tx", tx.getDouble(0));
        SmartDashboard.putNumber("limelight yaw", getYaw());
        SmartDashboard.putNumber("limelight ta", ta.getDouble(0));
    }

    /* ----- Getters & Setters ----- */

    public static LimelightSubsystem getInstance() {
        if (LL == null) {
            LL = new LimelightSubsystem();
        }

        return LL;
    }

    public double getTx() {
        return tx.getDouble(0);
    }

    public double getTa() {
        return ta.getDouble(0);
    }

    public double getYaw() {
        return botPose.getDoubleArray(new double[6])[4];
    }

    public Pose2d getPose() {
        double[] poseArray = botPoseField.getDoubleArray(new double[6]);
        Rotation2d rotation = new Rotation2d(poseArray[3], poseArray[4]);
        Pose2d out = new Pose2d(poseArray[0], poseArray[1], rotation);
        return out;
    }
    
}