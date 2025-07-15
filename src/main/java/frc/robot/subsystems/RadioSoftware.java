package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * RadioSoftware is a subsystem that manages musical playback using CTRE's Orchestra class.
 * It allows TalonFX motor controllers to play a .chrp music file ("Rickroll.chrp").
 */
public class RadioSoftware extends SubsystemBase {

    // Singleton instance of Orchestra used to manage musical instruments and playback
    private static Orchestra orchestra;

    // Singleton instance of RadioSoftware subsystem
    private static RadioSoftware RS;

    /**
     * Constructor - initializes the Orchestra and loads the music file.
     */
    public RadioSoftware() {
        orchestra = getOrchestraInstance(); // Retrieve (or initialize) the Orchestra instance
        var ok = orchestra.loadMusic("Rickroll.chrp"); // Load the specified .chrp music file
        System.out.println("STATUS: " + ok.isOK()); // Print status of the load operation
    }

    /**
     * Starts music playback.
     */
    public void playMusic() {
        orchestra.play();
        System.out.println("PLAY MUSIC YAYYAYAYAYAY"); // Fun confirmation message
    }

    /**
     * Pauses the currently playing music.
     */
    public void pauseMusic() {
        orchestra.pause();
    }

    /**
     * Stops the music completely.
     */
    public void stopMusic() {
        orchestra.stop();
    }

    /**
     * Adds a TalonFX motor to the Orchestra for musical output.
     * @param motorInstance the TalonFX motor controller to be added as an instrument
     */
    public void addMotor(TalonFX motorInstance) {
        orchestra.addInstrument(motorInstance);
        System.out.println("ADDED MOTOR"); // Confirmation message
    }

    /**
     * Retrieves the singleton instance of RadioSoftware.
     * Initializes it if it hasn't been created yet.
     * @return the singleton RadioSoftware instance
     */
    public static RadioSoftware getInstance() {
        if (RS == null) {
            RS = new RadioSoftware();
        }
        System.out.println("INSTANCE"); // Debug message
        return RS;
    }

    /**
     * Retrieves the singleton Orchestra instance.
     * Initializes it if it hasn't been created yet.
     * @return the singleton Orchestra instance
     */
    public static Orchestra getOrchestraInstance() {
        if (orchestra == null) {
            orchestra = new Orchestra();
        }
        return orchestra;
    }
}