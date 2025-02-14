package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoordinationSubsytem;

public class ManualPitchCommand extends Command {
    
    DoubleSupplier input;

    CoordinationSubsytem score = CoordinationSubsytem.getInstance();

    public ManualPitchCommand(DoubleSupplier input) {
        this.input = input;
        addRequirements(score);
    }

    @Override
    public void execute() {
        double stickValue = input.getAsDouble();
        if (Math.abs(stickValue) > 0.1) {
            score.pitchManualMovement(stickValue);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
