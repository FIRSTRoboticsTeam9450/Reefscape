package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
    NetworkTableEntry tx = limelight.getEntry("tx");
    NetworkTableEntry ty = limelight.getEntry("ty");
    NetworkTableEntry ta = limelight.getEntry("ta");
    NetworkTableEntry botPose = limelight.getEntry("botpose_targetspace");
    public static LimelightSubsystem yay;

    public static LimelightSubsystem getInstance() {
        if (yay == null) {
            yay = new LimelightSubsystem();
        }

        return yay;
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
    @Override
    public void periodic() {
        SmartDashboard.putNumber("limelight tx", tx.getDouble(0));
        SmartDashboard.putNumber("limelight ty", ty.getDouble(0));
        SmartDashboard.putNumber("limelight ta", ta.getDouble(0));
    }
    
}