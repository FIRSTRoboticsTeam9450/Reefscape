package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;

/**
 * Interface representing configuration settings for the robot.
 * Provides sensor offsets, control parameters, and positional targets.
 */
public interface RobotConfig {

    // ----- Swerve Module Angle Offsets -----

    /**
     * @return Offset angle for the front-left swerve module.
     */
    Angle getFrontLeftOffset();

    /**
     * @return Offset angle for the front-right swerve module.
     */
    Angle getFrontRightOffset();

    /**
     * @return Offset angle for the back-left swerve module.
     */
    Angle getBackLeftOffset();

    /**
     * @return Offset angle for the back-right swerve module.
     */
    Angle getBackRightOffset();


    // ----- Motor Control Gains -----

    /**
     * @return Slot0 configuration for the steering motors.
     */
    Slot0Configs getSteerGains();

    /**
     * @return Slot0 configuration for the driving motors.
     */
    Slot0Configs getDriveGains();


    // ----- Arm/Joint Calibration and Mechanics -----

    /**
     * @return Offset value for the elbow joint in degrees or radians.
     */
    double getElbowOffset();

    /**
     * @return Gear ratio for the elbow mechanism.
     */
    double getElbowRatio();


    // ----- Positional Targets -----

    /**
     * @return Ground-level position for the elbow joint.
     */
    double getElbowGroundPos();

    /**
     * @return Ground-level position for pitch control.
     */
    double getPitchGroundPos();

    /**
     * @return Net (combined) elevator position value.
     */
    double getElevatorNetPos();


    // ----- Object Detection Trigger Distances -----

    /**
     * @return Distance threshold for detecting coral (in meters or preferred units).
     */
    double getCoralTriggerDistance();

    /**
     * @return Distance threshold for detecting algae.
     */
    double getAlgaeTriggerDistance();


    // ----- Level 4 Predefined Positions -----

    /**
     * @return Pitch value preset for Level 4.
     */
    double getL4Pitch();

    /**
     * @return Elbow angle preset for Level 4.
     */
    double getL4Elbow();

    /**
     * @return Elevator height preset for Level 4.
     */
    double getL4Elevator();


    // ----- Miscellaneous -----

    /**
     * @return Whether to enable climber subsystem during runtime.
     */
    boolean getRunClimber();
}