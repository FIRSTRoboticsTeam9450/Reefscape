package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.debugging;

public class LimelightSubsystem extends SubsystemBase {

    //Limelight Data table
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
    //Limelight specific datas
    NetworkTableEntry tx = limelight.getEntry("tx");
    NetworkTableEntry ty = limelight.getEntry("ty");
    NetworkTableEntry ta = limelight.getEntry("ta");
    NetworkTableEntry botPose = limelight.getEntry("botpose_targetspace");
    NetworkTableEntry tid = limelight.getEntry("tid");
    NetworkTableEntry botPoseFieldBlue = limelight.getEntry("botpose_wpiblue");
    
    public static LimelightSubsystem LL; 

    double[] poseArray = new double[10];

    /* ----- Updaters ----- */

    @Override
    public void periodic() {
        if (debugging.LimelightDebugging) {
            SmartDashboard.putNumber("limelight tx", tx.getDouble(0));
            SmartDashboard.putNumber("limelight yaw", getYaw());
            SmartDashboard.putNumber("limelight ta", ta.getDouble(0));
            SmartDashboard.putNumber("pose ta", poseArray[9]);
        }
        Logger.recordOutput("Limelight Pose", getPose());
        poseArray = botPoseFieldBlue.getDoubleArray(new double[10]);
    }

    public LimelightSubsystem() {

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

    public int getTid() {
        return (int) tid.getDouble(0);
    }


    public double getYaw() {
        return botPose.getDoubleArray(new double[6])[4];
    }

    public double getTagCount() {
        return poseArray[7];
    }

    public Pose2d getPose() {
        Rotation2d rot = new Rotation2d(poseArray[5] * Math.PI / 180);
        Pose2d out = new Pose2d(poseArray[0], poseArray[1], rot);
        return out;
    }
    
}