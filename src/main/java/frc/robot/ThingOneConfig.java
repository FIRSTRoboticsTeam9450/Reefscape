package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;

public class ThingOneConfig implements RobotConfig {
    
    Angle frontLeftOffset = Rotations.of(-0.131103515625);
    Angle frontRightOffset = Rotations.of(-0.0537109375);
    Angle backLeftOffset = Rotations.of(0.273681640625);
    Angle backRightOffset = Rotations.of(0.47265625);

    Slot0Configs steerGains = new Slot0Configs()
        .withKP(80).withKI(0).withKD(1)
        .withKS(0).withKV(1.5).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);
    
    double elbowOffset = 0.32666015625;
    double elbowRatio = 92.99 / 90.0;
    
    double elbowGroundSetpoint = -6;
    double pitchGroundSetpoint = -155;
    
    double coralTriggerDistance = 115;
    double algaeTriggerDistance = 70;

    double elevatorNetPos = 39;

    // Practice field totes
    // double l4Pitch = -175.63;
    // double l4Elbow = 82;
    // double l4Elevator = 35;

    double l4Pitch = -182.63;
    double l4Elbow = 87;
    double l4Elevator = 35;

    boolean runClimber = true;

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
    public double getElbowGroundPos() {
        return elbowGroundSetpoint;
    }

    @Override
    public double getPitchGroundPos() {
        return pitchGroundSetpoint;
    }

    @Override
    public double getElevatorNetPos() {
        return elevatorNetPos;
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

    public boolean getRunClimber() {
        return runClimber;
    }

}