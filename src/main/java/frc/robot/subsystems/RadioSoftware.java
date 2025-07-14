package frc.robot.subsystems;

import java.io.File;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RadioSoftware extends SubsystemBase{
    
    private static Orchestra orchestra;

    private static RadioSoftware RS;

    public RadioSoftware() {
        orchestra = getOrchestraInstance();
        var ok = orchestra.loadMusic("Rickroll.chrp");
        System.out.println("STATUS: " + ok.isOK());
    }

    public void playMusic() {
        orchestra.play();
        System.out.println("PLAY MUSIC YAYYAYAYAYAY");
    }

    public void pauseMusic() {
        orchestra.pause();
    }

    public void stopMusic() {
        orchestra.stop();
    }

    public void addMotor(TalonFX motorInstance) {
        orchestra.addInstrument(motorInstance);
        System.out.println("ADDED MOTOR");
    }

    public static RadioSoftware getInstance() {
        if (RS == null) {
            RS = new RadioSoftware();
        }
        System.out.println("INSTANCE");
        return RS;
    }

    public static Orchestra getOrchestraInstance() {
        if (orchestra == null) {
            orchestra = new Orchestra();
        }
        return orchestra;
    }

}
