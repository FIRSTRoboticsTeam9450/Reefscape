package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class ThingOneConfig implements RobotConfig {
    
    public Angle frontLeftOffset = Rotations.of(-0.133056640625);
    public Angle frontRightOffset = Rotations.of(-0.0478515625);
    public Angle backLeftOffset = Rotations.of(0.092041015625);
    public Angle backRightOffset = Rotations.of(0.4794921875);

    @Override
    public Angle getFrontLeftOffset() {
        return frontLeftOffset;
    }

    @Override
    public Angle getFrontRightOffset() {
        return frontRightOffset;
    }

    @Override
    public Angle getBackLeftOffset() {
        return backLeftOffset;
    }

    @Override
    public Angle getBackRightOffset() {
        return backRightOffset;
    }



}
