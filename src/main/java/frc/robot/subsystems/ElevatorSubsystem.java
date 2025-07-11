package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.S1StateValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorIDs;

public class ElevatorSubsystem extends SubsystemBase{

    //Instance of Elevator Subsystem
    private static ElevatorSubsystem elev;

    //Motor instances
    private TalonFX leftMotor = new TalonFX(ElevatorIDs.kLeftMotorID, "CantDrive");
    private TalonFX rightMotor = new TalonFX(ElevatorIDs.kRightMotorID, "CantDrive");
    
    // class member variable
    
    private double currentVoltageOut;
    VoltageOut m_request = new VoltageOut(0);

    private double position;
    private double offset;
    private double setpoint;

    private final double HIGHSETPOINT = 25;
    private final double LOWSETPOINT = 2;
    
    private boolean atSetpoint;

    private double newVoltage = 0;
    // private boolean atLimit;

    double currentLimit = 110; // 100 is the max stator current pull

    private CANdi candi = new CANdi(ElevatorIDs.kCANdiID, "CantDrive");

    public static ElevatorSubsystem getInstance() {
        if (elev == null) {
            elev = new ElevatorSubsystem();
        }
        return elev;
    }

    private ElevatorSubsystem() {
        leftMotorConfig();
        rightMotorConfig();

        leftMotor.setControl (m_request);
        rightMotor.setControl(m_request);
        // rightMotor.setControl(m_request.withOutput(Volts.of(12.0)));
    }

    private void leftMotorConfig(){
        TalonFXConfigurator configurator = leftMotor.getConfigurator();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // the only difference
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = currentLimit;
        
        configurator.apply(config);
    }

    private void rightMotorConfig(){
        TalonFXConfigurator configurator = rightMotor.getConfigurator();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.defaultNeutral;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // the only difference
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = currentLimit;

        configurator.apply(config);
    }


    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    double volts = 0;
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(.5).div(Seconds.one()),
                    Volts.of(4),
                    Seconds.of(6), // Use default timeout (10 s)
                    (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix
                                                                                   // SignalLogger class
            ),
            new SysIdRoutine.Mechanism(
                    (volts) -> setVoltageOut(volts.in(Volts)),
                    // setVoltageOut(4),
                    null,
                    this));


   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    System.out.println("direction:" + direction);
   return m_sysIdRoutine.quasistatic(direction);
   }

   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.dynamic(direction);
   }
    /* ----- Updaters ----- */

    boolean pidControl;
    double pidTarget;
    double kp = .5;

    @Override
    public void periodic() {
        double rawPosition = leftMotor.getPosition().getValueAsDouble();
        position = rawPosition - offset;
        
        if (!pidControl){

            if (position > HIGHSETPOINT && currentVoltageOut > 0){
                pidTarget = HIGHSETPOINT;
                pidControl = true;
            }
            else if (position < LOWSETPOINT && currentVoltageOut <= 0){
                pidTarget = LOWSETPOINT;
                pidControl = true;
            }
        }
        else {
            if(pidTarget == HIGHSETPOINT && newVoltage < 0) {
                pidControl = false;
            }
            else if(pidTarget == LOWSETPOINT && newVoltage > 0) {
                pidControl = false;
            }
        }
        if (pidControl){
            currentVoltageOut = kp * (pidTarget - position);
        }

        leftMotor.setControl(m_request.withOutput(currentVoltageOut));
        rightMotor.setControl(m_request.withOutput(currentVoltageOut));
        
    }

    /* ----- Getters & Setters ----- */

    public double getPosition() {
        return position;
    }

    public void setSetpoint(double pSetpoint) {
        setpoint = pSetpoint;
    }

    public boolean atSetpoint() {
        return atSetpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setVoltageOut(double newVoltageOut){
        newVoltage = newVoltageOut;
        if (!pidControl && newVoltageOut != currentVoltageOut){
            System.out.println("CHANGED VOLTAGE: " + newVoltageOut);
            currentVoltageOut = newVoltageOut;
        }
    }

    // public void updateMotionMagic(double multiplier) {
    //     // if(multiplier < 0) {
    //     //     velocity = -20;
    //     // }
    //     // else if(multiplier == 0) {
    //     //     velocity = 0;
    //     // }
    //     // else{
    //     //     velocity = 20;
    //     // }
    //     //velocity = 40 * multiplier;
    //     //acceleration = 125 * multiplier;
    // }

    // private double dynamicUpdate(String desc, double current){
    //     double v = SmartDashboard.getNumber(desc,current);
    //     // if (v == current){
    //     //     SmartDashboard.putNumber(desc,current);
    //     // }
    //     return v;
    // }

    // private void dynamicPut(String desc, double current){
    //     SmartDashboard.putNumber(desc,current);
    // }

    // public void putParams(){
    //     dynamicPut("elev_velocity",velocity);
    //     dynamicPut("elev_acc", acceleration);
    //     dynamicPut("elev_jerk",jerk);

    //     dynamicPut("elev_currentlimit",currentLimit);

    //     dynamicPut("elev_kS",kS);
    //     dynamicPut("elev_kV",kV);
    //     dynamicPut("elev_kA",kA);
    //     dynamicPut("elev_kP",kP);
    //     dynamicPut("elev_kI",kI);
    //     dynamicPut("elev_kD",kD);
    //     dynamicPut("elev_kG",kG);
  
    //     leftMotorConfig();
    // }

    // public void updateParams(){
    //     velocity = dynamicUpdate("elev_velocity",velocity);
    //     acceleration = dynamicUpdate("elev_acc", acceleration);
    //     jerk = dynamicUpdate("elev_jerk",jerk);

    //     currentLimit = dynamicUpdate("elev_currentlimit",currentLimit);

    //     kS = dynamicUpdate("elev_kS",kS);
    //     kV = dynamicUpdate("elev_kV",kV);
    //     kA = dynamicUpdate("elev_kA",kA);
    //     kP = dynamicUpdate("elev_kP",kP);
    //     kI = dynamicUpdate("elev_kI",kI);
    //     kD = dynamicUpdate("elev_kD",kD);
    //     kG = dynamicUpdate("elev_kG",kG);
  
    //     leftMotorConfig();
    // }
}
