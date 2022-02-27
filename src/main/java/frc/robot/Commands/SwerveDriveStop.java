package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SwerveDriveSystem;

public class SwerveDriveStop extends CommandBase {
    private final SwerveDriveSystem swerveDriveSystem;
    public SwerveDriveStop(SwerveDriveSystem initialSwerveDriveSystem) {
        super();
        swerveDriveSystem = initialSwerveDriveSystem;
        addRequirements(swerveDriveSystem);
    }

    @Override
    public void execute() {
        swerveDriveSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}