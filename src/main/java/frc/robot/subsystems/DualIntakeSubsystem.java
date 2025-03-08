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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeIDS;
import frc.robot.Constants;

public class DualIntakeSubsystem extends SubsystemBase{

    /* ----- Instance of Subsystem ----- */
    private static DualIntakeSubsystem DI;

    int coralValidCount = 0;

    /* ----- Motors ----- */
    private TalonFX motor = new TalonFX(IntakeIDS.kDualIntakeMotorID, Constants.CTRE_BUS);
    /* ----- Laser Can ----- */
    CANrange coralRange = new CANrange(IntakeIDS.kDualIntakeCoralLaserID);
    CANrange algaeRange = new CANrange(IntakeIDS.kDualIntakeAlgaeLaserID);
    //private LaserCan.Measurement coralMeasurement;
    // LaserCANs are configured with a medianfilter, which means the last 3 reults are averaged together
    // this smooths out the output nicely
    //MedianFilter algaeMedianDistance = new MedianFilter(3);
    //MedianFilter coralMedianDistance = new MedianFilter(3);
    double coralLaserDistance;
    double algaeLaserDistance;

    boolean hasCoral;
    boolean hasAlgae;

    boolean lastHadCoral;
    boolean hasCoralNow;

    VoltageOut request = new VoltageOut(0).withEnableFOC(true);
    /* ----- Initialization ----- */

    /**
     * Configs motor
     * Configs LaserCans
     */
    private DualIntakeSubsystem() {
        CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
        rangeConfig.FovParams.FOVRangeX = 6.75;
        rangeConfig.FovParams.FOVRangeY = 6.75;

        coralRange.getConfigurator().apply(rangeConfig);
        algaeRange.getConfigurator().apply(rangeConfig);

        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        configurator.apply(config);

        //coralMeasurement = coralLaserCan.getMeasurement();
    }

    /* ----- Updaters ----- */

    /**
     * Updates the median distance of the laser can for the past 3 checks
     */
    public void updateLasers() {
        try {
            coralLaserDistance = coralRange.getDistance().getValueAsDouble() * 1000;
            algaeLaserDistance = algaeRange.getDistance().getValueAsDouble() * 1000;
            if (coralRange.getSignalStrength().getValueAsDouble() < 2500) {
                coralLaserDistance = 1000;
            }
            if (algaeRange.getSignalStrength().getValueAsDouble() < 2500) {
                algaeLaserDistance = 1000;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        hasCoralNow = coralLaserDistance < Constants.robotConfig.getCoralTriggerDistance();
        hasAlgae = algaeLaserDistance < Constants.robotConfig.getAlgaeTriggerDistance();
        
        if (hasCoralNow == lastHadCoral) {
            coralValidCount++;
            if (coralValidCount >= 2) {
                hasCoral = hasCoralNow;
            }
        } else {
            coralValidCount = 0;
        }

        lastHadCoral = hasCoralNow;
    
    }

    @Override
    public void periodic() {
        updateLasers();
        Logger.recordOutput("Reefscape/DualIntake/HasCoral", hasCoral);
        Logger.recordOutput("Reefscape/DualIntake/CoralLaserDistance", coralLaserDistance);
        Logger.recordOutput("Reefscape/DualIntake/CoralLaserStrength", coralRange.getSignalStrength().getValueAsDouble());

        Logger.recordOutput("Reefscape/DualIntake/HasAlgae", hasAlgae);
        Logger.recordOutput("Reefscape/DualIntake/AlgaeLaserDistance", algaeLaserDistance);
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

    public boolean hasAlgae() {
        return hasAlgae;
    }


    /**
     * sets voltage of motor to given voltage
     * @param voltage range of -12V to 12V
     */
    public void setVoltage(double voltage) {
        motor.setControl(request.withOutput(voltage));
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
