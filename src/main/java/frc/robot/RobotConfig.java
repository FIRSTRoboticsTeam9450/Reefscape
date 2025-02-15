package frc.robot;

import edu.wpi.first.units.measure.Angle;

public interface RobotConfig {
    
    public Angle getFrontLeftOffset();
    public Angle getFrontRightOffset();
    public Angle getBackLeftOffset();
    public Angle getBackRightOffset();

}
