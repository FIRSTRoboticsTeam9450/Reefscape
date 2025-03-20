package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeIDS;
import frc.robot.Constants;

public class DualIntakeSubsystem extends SubsystemBase{

    /* ----- Instance of Subsystem ----- */
    private static DualIntakeSubsystem DI;

    int coralValidCount = 0;

    CANrange laser = new CANrange(Constants.IntakeIDS.kDualIntakeCoralLaserID);

    /* ----- Motors ----- */
    private TalonFX motor = new TalonFX(IntakeIDS.kDualIntakeMotorID, Constants.CTRE_BUS);

    boolean hasCoral;
    boolean hasAlgae;

    boolean lastHadCoral;
    boolean hasCoralNow;

    double voltage;
    boolean atSpeed;

    VoltageOut request = new VoltageOut(0).withEnableFOC(true);

    CoordinationSubsytem score = CoordinationSubsytem.getInstance();
    /* ----- Initialization ----- */

    /**
     * Configs motor
     * Configs LaserCans
     */
    private DualIntakeSubsystem() {
        CANrangeConfiguration laserConfig = new CANrangeConfiguration();
        laserConfig.FovParams.FOVRangeX = 6.75;
        laserConfig.FovParams.FOVCenterY = 6.75;
        laserConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 10000;
        laserConfig.ProximityParams.ProximityThreshold = 0.09;
        
        laser.getConfigurator().apply(laserConfig);

        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 50;
        configurator.apply(config);

        //coralMeasurement = coralLaserCan.getMeasurement();
    }

    /* ----- Updaters ----- */

    /**
     * Updates the median distance of the laser can for the past 3 checks
     */
    public void updateLasers() {
        //hasCoralNow = coralLaserDistance < Constants.robotConfig.getCoralTriggerDistance();
        hasCoral = laser.getIsDetected().getValue();
        hasAlgae = score.getAlgae() && hasCoral;

        Logger.recordOutput("Reefscape/DualIntake/distance", laser.getDistance().getValueAsDouble());
        Logger.recordOutput("Reefscape/DualIntake/signal strength", laser.getSignalStrength().getValueAsDouble());
        Logger.recordOutput("Reefscape/DualIntake/detected", laser.getIsDetected().getValue());

        
        // if (hasCoralNow == lastHadCoral) {
        //     coralValidCount++;
        //     if (coralValidCount >= 2) {
        //         hasCoral = hasCoralNow;
        //     }
        // } else {
        //     coralValidCount = 0;
        // }

        //lastHadCoral = hasCoralNow;

        //double motorVelocity = motor.getVelocity().getValueAsDouble();

        // if (voltage > 1) {
        //     if (motorVelocity > 85) {
        //         atSpeed = true;
        //     }
        //     if (atSpeed) {
        //         if (motorVelocity < 70) {
        //             hasCoral = true;
        //             atSpeed = false;
        //             coralValidCount = 0;
        //         }
        //     }
        // }
        
        // Logger.recordOutput("Reefscape/DualIntake/Velocity", motorVelocity);
        // if (Math.abs(motorVelocity) > 75) {
        //     coralValidCount++;
        //     if (coralValidCount > 2) {
        //         hasCoral = false;
        //     }
        // } else {
        //     coralValidCount = 0;
        // }
    }

    @Override
    public void periodic() {
        updateLasers();
        Logger.recordOutput("Reefscape/DualIntake/HasCoral", hasCoral);

        Logger.recordOutput("Reefscape/DualIntake/HasAlgae", hasAlgae);
        Logger.recordOutput("Reefscape/DualIntake/MotorTemp", motor.getDeviceTemp().getValueAsDouble());
    }

    /* ----- Getters & Setters ----- */

    /**
     * returns the distance of the laser from the laser can
     * @return distance of laser
     */
    public boolean hasCoral() {
        return hasCoral;
    }

    public void setHasCoral(boolean hasCoral) {
        this.hasCoral = hasCoral;
    }

    public boolean hasAlgae() {
        return hasAlgae;
    }


    /**
     * sets voltage of motor to given voltage
     * @param voltage range of -12V to 12V
     */
    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motor.setControl(request.withOutput(voltage));
        atSpeed = false;
    }

    public void setMode(boolean coast) {
    }
    /**
     * returns instance of the subsystem
     * @return instance
     */
    public static DualIntakeSubsystem getInstance() {
        if (DI == null) {
            DI = new DualIntakeSubsystem();
        }
        return DI;
    }
    
}
