package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;

public class ThingOneConfig implements RobotConfig {
    
    Angle frontLeftOffset = Rotations.of(-0.133056640625);
    Angle frontRightOffset = Rotations.of(-0.0478515625);
    Angle backLeftOffset = Rotations.of(0.092041015625);
    Angle backRightOffset = Rotations.of(0.4794921875);

    Slot0Configs steerGains = new Slot0Configs()
        .withKP(80).withKI(0).withKD(1)
        .withKS(0).withKV(1.5).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);
    
    double elbowOffset = 0.127685546875;
    double elbowRatio = 82.09 / 90.0;
    
    double coralTriggerDistance = 15;
    double algaeTriggerDistance = 24;

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



}
