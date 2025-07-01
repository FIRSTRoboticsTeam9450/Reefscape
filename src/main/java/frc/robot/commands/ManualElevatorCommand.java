package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
    private DoubleSupplier input;
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    public ManualElevatorCommand(DoubleSupplier multiplier) {
        input = multiplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double multiplier = -input.getAsDouble();
        System.out.println(multiplier + "MULTIPLIER IS THIS");
        elevator.updateMotionMagic(multiplier);
    }

    @Override
    public boolean isFinished() {
        return false;
    } 
}
