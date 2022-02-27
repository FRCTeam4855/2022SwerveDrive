package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeMotorSS;

public class IntakeMotorForwardCom extends CommandBase {
    private final IntakeMotorSS intakeMotorSubsystem;

    public IntakeMotorForwardCom(IntakeMotorSS forwardSubsystem) {
        intakeMotorSubsystem = forwardSubsystem;
        addRequirements(intakeMotorSubsystem);
    }

    @Override
    public void initialize() {
        intakeMotorSubsystem.forwardIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}