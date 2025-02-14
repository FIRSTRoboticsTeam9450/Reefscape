package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climbers, they go up and down... is fun :)
 */
public class ClimbSubsystem extends SubsystemBase {
    
    /* ----- Motor ----- */
    private SparkFlex climb = new SparkFlex(20, MotorType.kBrushless);

    /* ----------- Initializaton ----------- */

    public ClimbSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* ----------- Setters & Getters ----------- */

    public void setVoltage(double voltage) {
        climb.setVoltage(voltage);
    }

}
