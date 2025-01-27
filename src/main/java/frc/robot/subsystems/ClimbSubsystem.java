package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    
    SparkFlex climb = new SparkFlex(20, MotorType.kBrushless);

    public ClimbSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltage(double voltage) {
        climb.setVoltage(voltage);
    }

}
