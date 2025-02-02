package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DualIntakeSubsystem;

public class OuttakeCommand extends Command {

    DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();

    Timer timer = new Timer();

    public OuttakeCommand() {
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        if (intake.getAlgaeLaserDistance() < 25) {
            intake.setVoltage(5);
        } else if (intake.getCoralLaserDistance() < 25) {
            intake.setVoltage(-5);
        }

        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 1;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }


    
}
