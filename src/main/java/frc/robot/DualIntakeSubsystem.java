package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeIDS;

public class DualIntakeSubsystem extends SubsystemBase{

    /* ----- Instance of Subsystem ----- */
    private static DualIntakeSubsystem DI;

    /* ----- Motors ----- */
    private TalonFX motor = new TalonFX(59, Constants.CTRE_BUS);

    /* ----- Laser Can ----- */
    private LaserCan coralLaserCan;
    private LaserCan algaeLaserCan;
    private LaserCan.Measurement coralMeasurement;
    private LaserCan.Measurement algaeMeasurement;
    // LaserCANs are configured with a medianfilter, which means the last 3 reults are averaged together
    // this smooths out the output nicely
    MedianFilter medianDistance = new MedianFilter(3);
    double coralLaserDistance;
    double algaeLaserDistance;

    /* ----- Initialization ----- */

    /**
     * Configs motor
     * Configs LaserCans
     */
    public DualIntakeSubsystem() {
        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configurator.apply(config);

        //LaserCan settings
        coralLaserCan = new LaserCan(IntakeIDS.kDualIntakeCoralLaserID);
        algaeLaserCan = new LaserCan(IntakeIDS.kDualIntakeAlgaeLaserID);
        try {
            coralLaserCan.setRangingMode(LaserCan.RangingMode.LONG);
            algaeLaserCan.setRangingMode(LaserCan.RangingMode.LONG);

            coralLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 16, 16));
            algaeLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 16, 16));
            coralLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            algaeLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Error during Laser Can Configuration: " + e);
        }

        coralMeasurement = coralLaserCan.getMeasurement();
    }

    /* ----- Updaters ----- */

    /**
     * Updates the median distance of the laser can for the past 3 checks
     */
    public void updateLasers() {
        try {
            coralMeasurement = coralLaserCan.getMeasurement();
            algaeMeasurement = algaeLaserCan.getMeasurement();
            if (coralMeasurement != null && coralMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                coralLaserDistance = medianDistance.calculate(coralMeasurement.distance_mm);
            }
            if (algaeMeasurement != null && algaeMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                algaeLaserDistance = medianDistance.calculate(algaeMeasurement.distance_mm);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        updateLasers();
        SmartDashboard.putNumber("Reefscape/DualIntake/CoralLaserDistance", coralLaserDistance);
        SmartDashboard.putNumber("Reefscape/DualIntake/AlgaeLaserDistance", algaeLaserDistance);
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
        motor.setVoltage(voltage);
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
