package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;

public class ThingTwoConfig implements RobotConfig {
    
    Angle frontLeftOffset = Rotations.of(-0.4580078125);
    Angle frontRightOffset = Rotations.of(-0.34423828125);
    Angle backLeftOffset = Rotations.of(0.149169921875);
    Angle backRightOffset = Rotations.of(0.366943359375);

    Slot0Configs steerGains = new Slot0Configs()
        .withKP(80).withKI(0).withKD(1)
        .withKS(0).withKV(1.5).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);
    
    double elbowOffset = -0.08154296875;
    double elbowRatio = 92.99 / 90.0;
    
    double coralTriggerDistance = 0.14;
    double algaeTriggerDistance = 30;

    double l4Pitch = -143.63;
    double l4Elbow = 73.14;
    double l4Elevator = 35;

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

    @Override
    public Slot0Configs getSteerGains() {
        return steerGains;
    }

    @Override
    public Slot0Configs getDriveGains() {
        return driveGains;
    }

    @Override
    public double getElbowOffset() {
        return elbowOffset;
    }

    @Override
    public double getElbowRatio() {
        return elbowRatio;
    }

    @Override
    public double getCoralTriggerDistance() {
        return coralTriggerDistance;
    }

    @Override
    public double getAlgaeTriggerDistance() {
        return algaeTriggerDistance;
    }

    @Override
    public double getL4Pitch() {
        return l4Pitch;
    }
    
    @Override
    public double getL4Elbow() {
        return l4Elbow;
    }

    @Override
    public double getL4Elevator() {
        return l4Elevator;
    }

}
