package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
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
    int loops = 0;

    /* ----- Motors ----- */
    private TalonFX motor = new TalonFX(IntakeIDS.kDualIntakeMotorID, Constants.CTRE_BUS);
    /* ----- Laser Can ----- */
    private LaserCan coralLaserCan;
    private LaserCan algaeLaserCan;
    private LaserCan.Measurement coralMeasurement;
    private LaserCan.Measurement algaeMeasurement;
    // LaserCANs are configured with a medianfilter, which means the last 3 reults are averaged together
    // this smooths out the output nicely
    MedianFilter algaeMedianDistance = new MedianFilter(3);
    MedianFilter coralMedianDistance = new MedianFilter(3);
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
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        configurator.apply(config);

        //LaserCan settings
        coralLaserCan = new LaserCan(IntakeIDS.kDualIntakeCoralLaserID);
        algaeLaserCan = new LaserCan(IntakeIDS.kDualIntakeAlgaeLaserID);
        try {
            coralLaserCan.setRangingMode(LaserCan.RangingMode.LONG);
            algaeLaserCan.setRangingMode(LaserCan.RangingMode.LONG);
            coralLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 8, 8));
            algaeLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 8, 8));
            coralLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
            algaeLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
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
            //algaeMeasurement = algaeLaserCan.getMeasurement();
            if (coralMeasurement != null && coralMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                coralLaserDistance = coralMedianDistance.calculate(coralMeasurement.distance_mm);
            } else {
                coralLaserDistance = -50;
            }
            // if (algaeMeasurement != null && algaeMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            //     algaeLaserDistance = algaeMedianDistance.calculate(algaeMeasurement.distance_mm);
            // } else {
            //     algaeLaserDistance = 10000;
            // }
            //Logger.recordOutput("Co", null);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        updateLasers();
        loops++;
        SmartDashboard.putNumber("Reefscape/DualIntake/Loops", loops);
        Logger.recordOutput("Reefscape/DualIntake/CoralLaserDistance", coralLaserDistance);
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
