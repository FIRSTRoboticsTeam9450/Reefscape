package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ScoringCommand extends Command{ // DONT USE
    private double elevatorPos;
    private double elbowPos;
    private double pitchPos;
    private double rollPos;
    public ScoringCommand(double elevatorPos, double elbowPos, double pitchPos, double rollPos) {
        this.elevatorPos = elevatorPos;
        this.elbowPos = elbowPos;
        this.pitchPos = pitchPos;
        this.rollPos = rollPos;
    }

    @Override
    public void execute() {
        ElevatorCommand elevator = new ElevatorCommand(elevatorPos);
        ElbowCommand elbow = new ElbowCommand(elbowPos);
    }
}
