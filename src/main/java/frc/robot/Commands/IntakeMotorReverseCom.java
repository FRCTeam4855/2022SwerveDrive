package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeMotorSS;

public class IntakeMotorReverseCom extends CommandBase {
    private final IntakeMotorSS intakeMotorSubsystem;

    public IntakeMotorReverseCom(IntakeMotorSS reverseSubsystem) {
        intakeMotorSubsystem = reverseSubsystem;
        addRequirements(intakeMotorSubsystem);
    }

    @Override
    public void initialize() {
        intakeMotorSubsystem.reverseIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}