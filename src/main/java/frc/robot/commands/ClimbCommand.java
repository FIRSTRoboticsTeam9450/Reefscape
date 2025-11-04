package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.RobotConstants.*;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DualIntakeSubsystem;

public class ClimbCommand extends Command{

    private ClimbSubsystem CS = ClimbSubsystem.getInstance();

    private DualIntakeSubsystem intake = DualIntakeSubsystem.getInstance();
    
    private ClimbPos setpoint;

    public ClimbCommand(ClimbPos setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        if (setpoint != ClimbPos.STORE) {
            if (CS.getSetpoint() == ClimbPos.CLIMBING || CS.getSetpoint() == ClimbPos.STORE) {
                setpoint = ClimbPos.ENGAGING;
            } else if (CS.getSetpoint() == ClimbPos.ENGAGING) {
                setpoint = ClimbPos.CLIMBING;
            }
        }
        CS.setSetpoint(setpoint);
        if (CS.getSetpoint() == ClimbPos.CLIMBING) {
            new CoordinationCommand(ScoringPos.START)
            .andThen(new InstantCommand(() -> intake.setVoltage(0))).schedule();;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
