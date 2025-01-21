package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.Constants.ScorePos;
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

    ScorePos position = ScorePos.START;

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
        pitchEncoder = diffWrist.getPitchAngle();
        rollEncoder = diffWrist.getRollAngle();
        elbowEncoder = elbow.getAngle();
    
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
        pitchEncoder = diffWrist.getPitchAngle();
        rollEncoder = diffWrist.getRollAngle();
        elbowEncoder = elbow.getAngle();
        elevatorEncoder = elevator.getPosition();

        updatePosition();
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

    public void goToGroundIntakeCoral() { // elbow -12, pitch -43, roll 0
        elbow.setSetpoint(-18);
        diffWrist.setPitchSetpoint(-108);
        if (elbowEncoder < 30) {
            diffWrist.setRollSetpoint(0);
        }
        if (pitchEncoder > -47) {
            elevator.setSetpoint(0);
        }
    }

    public void goToGroundIntakeAlgae() { //old:  elbow -43, pitch -38, roll 0
                                          //new: elbow -48, pitch -130, roll 0
        elevator.setSetpoint(6);
        if (elevatorEncoder > 5) {
            diffWrist.setPitchSetpoint(-130);
            elbow.setSetpoint(-48);
        }
        if (elbowEncoder < 30) {
            diffWrist.setRollSetpoint(0);
        }
         //old: 6   
    }

    public void goToStoreAlgae() { // elbow 75, pitch -65, roll 0
        //p: -75
        //elev: 7
        //elbow: 30
        //roll: 0;
        elevator.setSetpoint(8);
        diffWrist.setRollSetpoint(0);

        double elevPos = elevatorEncoder;
        double elevSetpoint = elevator.getSetpoint();

        if ((elevPos > elevSetpoint - 0.05) && (elevPos < elevSetpoint + 0.05)) {
            elbow.setSetpoint(30);
            diffWrist.setPitchSetpoint(-75);
        }
    }

    public void goToStart() { // elbow 85, pitch -6, roll -90

        diffWrist.setPitchSetpoint(-91);
        if (rollEncoder < 0) {
            diffWrist.setRollSetpoint(-85);
        } else if (rollEncoder > 0) {
            diffWrist.setRollSetpoint(85);
        }
        if (pitchEncoder > 5) {
            elbow.setSetpoint(0);
        } else if (rollEncoder < -60 ) {
            elbow.setSetpoint(97);
        }
        elevator.setSetpoint(0);
    }

    public void goToStoreCoral() { //Elb: 80, pitch: -22, roll: -90
        diffWrist.setPitchSetpoint(-75);
        if (rollEncoder < 0) {
            diffWrist.setRollSetpoint(-85);
        } else if (rollEncoder > 0) {
            diffWrist.setRollSetpoint(85);
        }
        if (rollEncoder < -60) {
            elbow.setSetpoint(60);
        }
    }


    public void goToScoreCoral() {//Elb: 25, Pitch: 7, roll: -90
        diffWrist.setPitchSetpoint(-52);
        diffWrist.setRollSetpoint(0);
        elbow.setSetpoint(-3.7);
    }

    public void goToPosition(ScorePos pos) {
        this.position = pos;
    }

    public void updatePosition() {
        if (position == ScorePos.INTAKE_CORAL) {
            goToGroundIntakeCoral();
        } else if (position == ScorePos.START) {
            goToStart();
        } else if (position == ScorePos.STORE_CORAL) {
            goToStoreCoral();
        } else if (position == ScorePos.SCORE_CORAL) {
            goToScoreCoral();
        } else if (position == ScorePos.INTAKE_ALGAE) {
            goToGroundIntakeAlgae();
        } else if (position == ScorePos.STORE_ALGAE) {
            goToStoreAlgae();
        }
    }
    
}
