package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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

    int count;

    /* ----- Motors ----- */
    private TalonFX motor = new TalonFX(IntakeIDS.kDualIntakeMotorID, Constants.CTRE_BUS);
    /* ----- Laser Can ----- */
    private LaserCan coralLaserCan;
    private LaserCan algaeLaserCan;
    //private LaserCan.Measurement coralMeasurement;
    private LaserCan.Measurement algaeMeasurement;
    // LaserCANs are configured with a medianfilter, which means the last 3 reults are averaged together
    // this smooths out the output nicely
    //MedianFilter algaeMedianDistance = new MedianFilter(3);
    //MedianFilter coralMedianDistance = new MedianFilter(3);
    double coralLaserDistance;
    double algaeLaserDistance;

    VoltageOut request = new VoltageOut(0).withEnableFOC(true);
    /* ----- Initialization ----- */

    /**
     * Configs motor
     * Configs LaserCans
     */
    private DualIntakeSubsystem() {
        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        configurator.apply(config);

        //LaserCan settings
        coralLaserCan = new LaserCan(IntakeIDS.kDualIntakeCoralLaserID);
        algaeLaserCan = new LaserCan(IntakeIDS.kDualIntakeAlgaeLaserID);
        try {
            coralLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            algaeLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            coralLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            algaeLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            coralLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            algaeLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Error during Laser Can Configuration: " + e);
        }

        //coralMeasurement = coralLaserCan.getMeasurement();
    }

    /* ----- Updaters ----- */

    /**
     * Updates the median distance of the laser can for the past 3 checks
     */
    public void updateLasers() {
        try {
            LaserCan.Measurement coralMeasurement = coralLaserCan.getMeasurement();
            LaserCan.Measurement algaeMeasurement = algaeLaserCan.getMeasurement();
            if (coralMeasurement != null && coralMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                coralLaserDistance = coralMeasurement.distance_mm;
            } else {
                coralLaserDistance = 10000;
            }
            if (algaeMeasurement != null && algaeMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                algaeLaserDistance = algaeMeasurement.distance_mm;
            } else {
                algaeLaserDistance = 10000;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        updateLasers();
        Logger.recordOutput("Reefscape/DualIntake/CoralLaserDistance", coralLaserDistance);
        Logger.recordOutput("Reefscape/DualIntake/AlgaeLaserDistance", algaeLaserDistance);
        Logger.recordOutput("Reefscape/DualIntake/MotorTemp", motor.getDeviceTemp().getValueAsDouble());
    }

    /* ----- Getters & Setters ----- */

    /**
     * returns the distance of the laser from the laser can
     * @return distance of laser
     */
    public double getCoralLaserDistance() {
        return coralLaserDistance;
    }

    public double getAlgaeLaserDistance() {
        return algaeLaserDistance;
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
