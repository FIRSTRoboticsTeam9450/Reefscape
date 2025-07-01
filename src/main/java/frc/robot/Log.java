package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Log {
    String desc;
    double value;
    public Log(String desc, double value) {
        this.desc = desc;
        this.value = value;
        SmartDashboard.putNumber(desc, value);
    }

    public double dynamicGet() {
        value = SmartDashboard.getNumber(desc, value);
        return SmartDashboard.getNumber(desc,value);
    }

}