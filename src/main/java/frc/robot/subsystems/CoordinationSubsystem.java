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
    double pitchEncoderTarget;
    double rollEncoderTarget;
    double elevatorEncoderTarget;
    double elbowEncoderTarget;
    boolean firstTime;
    boolean goToPosition;
    boolean startPitch;
    boolean startElevator;
    boolean startRoll;
    boolean start = true;
    /* ----- Initialization ----- */
    public CoordinationSubsystem() {
        pitchEncoder = diffWrist.getPitchEncoder();
        rollEncoder = diffWrist.getRollEncoder();
        elbowEncoder = elbow.getEncoder();
    
    }

    /* ----- Arm Positions ----- */

    public boolean checkSafe() {
        if(firstTime) {
            firstTime = false;
            if(elbowEncoder > Constants.ArmConstraints.kElbowMax) {
                elbowEncoderTarget = Constants.ArmConstraints.kElbowMax;
            }
            else if(elbowEncoder < Constants.ArmConstraints.kElbowMin) {
                elbowEncoderTarget = Constants.ArmConstraints.kElbowMin;
            }

        }
        else {
            if(elbow.atSetpoint()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Moves the arm into source position
     */
    public void source() { 
        rollEncoderTarget = Constants.ArmConstraints.kRollVerticalAlgaeBottom;
        elevatorEncoderTarget = Constants.ArmPositions.kSourceElevator;
        elbowEncoderTarget = Constants.ArmPositions.kSourceElbow;
        pitchEncoderTarget = Constants.ArmPositions.kSourcePitch;
        goToPosition = true;
    }

    @Override
    public void periodic() {
        pitchEncoder = diffWrist.getPitchEncoder();
        rollEncoder = diffWrist.getRollEncoder();
        elbowEncoder = elbow.getEncoder();
        if(goToPosition) {
            boolean safe = checkSafe();
            if(safe && start) {
                startRoll = true;
                start = false;
            }
            if(startRoll) {
                diffWrist.setRollSetpoint(rollEncoderTarget);
            }
            if(diffWrist.atPitchSetpoint() && startRoll) {
                startRoll = false;
                startElevator = true;
            }
            if(startElevator) {
                elevator.setSetpoint(elevatorEncoderTarget);
            }
            if(elevator.atSetpoint() && startElevator) {
                startElevator = false;
                startPitch = true;
            }
            if(startPitch) {
                diffWrist.setPitchSetpoint(pitchEncoderTarget);
            }
            if(diffWrist.atPitchSetpoint()) {
                goToPosition = false;
                startPitch = false;
                startElevator = false;
                startRoll = false;
                start = true;

            }
        }

    }

    /**
     * Moves the arm into the correct position for ground intaking coral or algae
     * @param coral True if the intake is for coral, False if the intake is for algae
     */
    public void groundIntake(boolean coral) {
        if(coral) {
            if(rollEncoder != Constants.ArmConstraints.kRollVerticalCoralBottom) {
                if(elbowEncoder > Constants.ArmConstraints.kElbowMax) {
                    elbow.setSetpoint(Constants.ArmConstraints.kElbowMax);
                    // Switch roll encoder to vertical
                }
                else if(elbowEncoder < Constants.ArmConstraints.kElbowMin) {
                    elbow.setSetpoint(Constants.ArmConstraints.kElbowMin);
                    // Switch roll encoder to vertical
                }
            }
        }
        else if(!coral) {
            
            if(rollEncoder != Constants.ArmConstraints.kRollVerticalAlgaeBottom) {
                if(elbowEncoder > Constants.ArmConstraints.kElbowMax) {
                    elbow.setSetpoint(Constants.ArmConstraints.kElbowMax);
                    // Switch roll encoder to vertical
                }
                else if(elbowEncoder < Constants.ArmConstraints.kElbowMin) {
                    elbow.setSetpoint(Constants.ArmConstraints.kElbowMin);
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




    
}
