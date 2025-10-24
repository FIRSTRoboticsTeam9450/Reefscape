package frc.robot.subsystems;

import java.time.Period;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

// import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.*;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants;
// import frc.robot.Constants.ClimberIDs;
// import frc.robot.Constants.ElevatorIDs;
// import frc.robot.Constants.WristIDs;
// import frc.robot.Constants.debugging;

// /**
//  * Climbers, they go up and down... is fun :)
//  * <p>
//  * They do be lifting the hefty robot
//  * 
//  * <p> Grabbing Cage: 0.9 </p>
//  * <p> Climbing: 0.3 </p>
//  * <p> Store: 0.1 </p>
//  */
// public class ClimbSubsystem extends SubsystemBase {

//     /* -------- Instance -------- */
//     private static ClimbSubsystem CS;

//     /* -------- Components -------- */
//     // private final SparkFlex climb = new SparkFlex(ClimberIDs.kMotorID, MotorType.kBrushless);
//     // private final SparkAbsoluteEncoder encoder = climb.getAbsoluteEncoder();
//     private final TalonFX climb =  new TalonFX(ClimberIDs.kMotorID, Constants.CTRE_BUS);
//     private final CANcoder encoder = new CANcoder(ClimberIDs.kEncoderID, Constants.CTRE_BUS);
//     private final PIDController pid = new PIDController(55, 0, 0.5);
//     private final boolean runClimber = Constants.robotConfig.getRunClimber();
//     //private int climbState = 1; //1-stored, 2-deployed, 3-retract

//     private double maxVolts = 12;

//     /* -------- Constructor -------- */
//     private ClimbSubsystem() {
//         pid.setSetpoint(0.1); // Store position
//         configureEncoder();
//         motorConfig();
//         // climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     private void configureEncoder() {
//         CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
//         cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.1;
//         cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
//         encoder.getConfigurator().apply(cc_cfg);
//     }

//     // Configure TalonFX motor settings
//     public void motorConfig() {
//         TalonFXConfiguration config = new TalonFXConfiguration();

//         // Feedback configuration
//         config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
//         config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

//         // Motor output settings
//         config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
//         climb.getConfigurator().apply(config);
//     }

//     /* -------- Periodic Update -------- */
//     @Override
//     public void periodic() {
//         Logger.recordOutput("Reefscape/Climbers/Motor Revolutions", encoder.getAbsolutePosition().getValueAsDouble());
//         if (runClimber) {
//             System.out.println(encoder.getAbsolutePosition().getValueAsDouble());
//             // double voltage = updatePIDs(encoder.getPosition().getValueAsDouble());
//             // setVoltage(-voltage);
//             // if(climbState==1){//stored
//             //     setSetpoint(0);
//             // }else if(climbState == 2){//deployed
//             //     setSetpoint(1);
//             // }else{//retract/climb
//             //     setSetpoint(2);
//             // }

//             if (debugging.ClimberPos) {
//                 //Logger.recordOutput("Reefscape/Climbers/Motor Revolutions", encoder.getAbsolutePosition().getValueAsDouble());
//                 Logger.recordOutput("Reefscape/Climbers/PID Setpoint", pid.getSetpoint());
//                 //Logger.recordOutput("Reefscape/Climbers/Voltage", voltage);
//             }
//         }
//     }

//     /* -------- PID Helpers -------- */
//     public double updatePIDs(double pos) {
//         double voltage = pid.calculate(pos);
//         return MathUtil.clamp(voltage, -maxVolts, maxVolts);
//     }

//     /* -------- Setters -------- */
//     public void setVoltage(double voltage) {
//         System.out.println("voltage" + voltage);
//         climb.setVoltage(voltage);
//     }

//     public void setSetpoint(double setpoint) {
//         pid.setSetpoint(setpoint);
//     }

//     public void setMaxVolts(double maxVolts) {
//         this.maxVolts = Math.abs(maxVolts);
//     }

//     /* -------- Singleton -------- */
//     public static ClimbSubsystem getInstance() {
//         if (CS == null) {
//             CS = new ClimbSubsystem();
//         }
//         return CS;
//     }
// }

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.debugging;


public class ClimbSubsystem extends SubsystemBase {

    public static ClimbSubsystem instance;

    private SparkFlex climbMotor = new SparkFlex(Constants.ClimberIDs.kMotorID, MotorType.kBrushless);
    private final SparkAbsoluteEncoder climbEncoder = climbMotor.getAbsoluteEncoder();
    private final PIDController pid = new PIDController(55, 0, 0.5);
    private final boolean runClimber = Constants.robotConfig.getRunClimber();
    // private TalonFX climbMotor = new TalonFX(Constants.ClimberIDs.kMotorID, Constants.CTRE_BUS);
    // private CANcoder climbEncoder = new CANcoder(Constants.ClimberIDs.kEncoderID, Constants.CTRE_BUS);
    // private TalonFX temporaryEncoderTestingThingyMagigySoICanSeeIfTheEncoderNeedsToBeATalonFXInsteadOfACANCoder = new TalonFX(28, Constants.CTRE_BUS);


    private double maxVolts = 4;

    public ClimbSubsystem() {
        configuration();
        pid.setSetpoint(0.153);
    }

    public void configuration() {
            
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* -------- Periodic Update -------- */
    @Override
    public void periodic() {
        if (runClimber) {
            double voltage = updatePIDs(climbEncoder.getPosition());
            setVoltage(-voltage);

            if (debugging.ClimberPos) {
                Logger.recordOutput("Reefscape/Climbers/Motor Revolutions", climbEncoder.getPosition());
                Logger.recordOutput("Reefscape/Climbers/PID Setpoint", pid.getSetpoint());
                Logger.recordOutput("Reefscape/Climbers/Voltage", voltage);
            }
        }
    }

    /* -------- PID Helpers -------- */
    public double updatePIDs(double pos) {
        double voltage = pid.calculate(pos);
        return MathUtil.clamp(voltage, -maxVolts, maxVolts);
    }

    /* -------- Setters -------- */

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        climbMotor.setVoltage(voltage);
    }

    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
    }

    public void setMaxVolts(double maxVolts) {
        this.maxVolts = Math.abs(maxVolts);
    }

    public static ClimbSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimbSubsystem();
        }
        return instance;
    }

}