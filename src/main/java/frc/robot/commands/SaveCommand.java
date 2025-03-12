package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoordinationSubsytem;
import frc.robot.subsystems.DiffWristSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SaveCommand extends Command {
    
    DoubleSupplier input;

    CoordinationSubsytem score = CoordinationSubsytem.getInstance();
    DiffWristSubsystem wrist = DiffWristSubsystem.getInstance();
    ElbowSubsystem elbow = ElbowSubsystem.getInstance();
    ElevatorSubsystem elev = ElevatorSubsystem.getInstance();

    public SaveCommand() {
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("New roll position", wrist.getRollAngle());
        SmartDashboard.putNumber("New pitch position", wrist.getPitchAngle());
        SmartDashboard.putNumber("New elbow position", elbow.getAngle());
        SmartDashboard.putNumber("New elevator position", elev.getPosition());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
