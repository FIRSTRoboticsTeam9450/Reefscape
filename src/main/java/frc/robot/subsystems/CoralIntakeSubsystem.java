package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase{
    
    //Instance of Subsystem object
    private static CoralIntakeSubsystem CI;

    public static CoralIntakeSubsystem getInstance() {
        if (CI == null) {
            CI = new CoralIntakeSubsystem();
        }
        return CI;
    }
}