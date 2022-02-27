package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeMotorSS;

public class IntakeMotorStopCom extends CommandBase {
    private final IntakeMotorSS intakeMotorSubsystem;

    public IntakeMotorStopCom(IntakeMotorSS stopSubsystem) {
        intakeMotorSubsystem = stopSubsystem;
        addRequirements(intakeMotorSubsystem);
    }

    @Override
    public void initialize() {
        intakeMotorSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}