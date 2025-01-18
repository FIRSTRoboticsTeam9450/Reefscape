package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElbowSubsystem;
import frc.robot.Constants;

public class CoordinationSubsystem extends SubsystemBase {
    // Subsystems
    DiffWristSubsystem diffWrist = DiffWristSubsystem.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ElbowSubsystem elbow = ElbowSubsystem.getInstance();
    double pitchEncoder;
    double rollEncoder;
    double elevatorEncoder;
    double elbowEncoder;
    /* ----- Initialization ----- */
    public CoordinationSubsystem() {
        pitchEncoder = diffWrist.getPitchEncoder();
        rollEncoder = diffWrist.getRollEncoder();
        elbowEncoder = elbow.getEncoder();
    
    }

    /* ----- Arm Positions ----- */

    /**
     * Moves the arm into source position
     */
    public void source() { 
        if(rollEncoder == Constants.ArmConstraints.kRollHorizontal) {
            if(elbowEncoder > Constants.ArmConstraints.kElbowMax) {
                // Move down
                // Switch roll encoder to vertical
            }
            else if(elbowEncoder < Constants.ArmConstraints.kElbowMin) {
                // Move up
                // Switch roll encoder to vertical
            }
        }
        // Move elevator to right place!
        if(elbowEncoder < Constants.ArmPositions.kSourceElbow) {
            // move up 
        }
        else if(elbowEncoder > Constants.ArmPositions.kSourceElbow) {
            // move down;
        }
        // Move pitch to right place!
    }

    /**
     * Moves the arm into the correct position for ground intaking coral or algae
     * @param coral True if the intake is for coral, False if the intake is for algae
     */
    public void groundIntake(boolean coral) {
        if(coral) {
            if(rollEncoder != Constants.ArmConstraints.kRollVerticalCoralBottom) {
                if(elbowEncoder > Constants.ArmConstraints.kElbowMax) {
                    // Move down
                    // Switch roll encoder to vertical
                }
                else if(elbowEncoder < Constants.ArmConstraints.kElbowMin) {
                    // Move up
                    // Switch roll encoder to vertical
                }
            }
        }
        else if(!coral) {
            
            if(rollEncoder != Constants.ArmConstraints.kRollVerticalAlgaeBottom) {
                if(elbowEncoder > Constants.ArmConstraints.kElbowMax) {
                    // Move down
                    // Switch roll encoder to vertical
                }
                else if(elbowEncoder < Constants.ArmConstraints.kElbowMin) {
                    // Move up
                    // Switch roll encoder to vertical
                }
            }
        }
        // Move elevator to right place!
        if(elbowEncoder < Constants.ArmPositions.kSourceElbow) {
            // move up 
        }
        else if(elbowEncoder > Constants.ArmPositions.kSourceElbow) {
            // move down;
        }
        // Move pitch to right place!

    }

    /**
     * Moves the arm to the right position to score
     * @param branch The level you're scoring on (1 is trough and 4 is the highest)
     */
    public void reefScore(int branch) {
        if(branch == 4) {
            if(rollEncoder != Constants.ArmConstraints.kRollVerticalCoralBottom) {
                if(elbowEncoder > Constants.ArmConstraints.kElbowMax) {
                    // Move down
                    // Switch roll encoder to horizontal
                }
                else if(elbowEncoder < Constants.ArmConstraints.kElbowMin) {
                    // Move up
                    // Switch roll encoder to vertical
                }
            }

        }
        else {

        }
    }

    @Override
    public void periodic() {
        pitchEncoder = diffWrist.getPitchEncoder();
        rollEncoder = diffWrist.getRollEncoder();
        elbowEncoder = elbow.getEncoder();
    }



    
}
