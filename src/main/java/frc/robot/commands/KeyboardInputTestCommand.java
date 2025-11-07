package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class KeyboardInputTestCommand extends Command{

    double endCount = 0;

    public KeyboardInputTestCommand() {

    }

    @Override
    public void initialize() {
        endCount = 0;
    }

    @Override
    public void execute() {
        System.out.println("BUTTON WORKS AS A KEYBIND / TRIGGERED COMMAND WHEN BUTTON PRESSED");
        endCount++;
    }

    @Override
    public boolean isFinished() {
        return endCount > 150;
    }
    
}
