package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.units.measure.Angle;

public interface RobotConfig {
    
    public Angle getFrontLeftOffset();
    public Angle getFrontRightOffset();
    public Angle getBackLeftOffset();
    public Angle getBackRightOffset();

    public Slot0Configs getSteerGains();
    public Slot0Configs getDriveGains();

    public double getElbowOffset();
    public double getElbowRatio();
    
    public double getElbowGroundPos();
    public double getPitchGroundPos();

    public double getElevatorNetPos();

    public double getCoralTriggerDistance();
    public double getAlgaeTriggerDistance();

    public double getL4Pitch();
    public double getL4Elbow();
    public double getL4Elevator();

    public boolean getRunClimber();

}
