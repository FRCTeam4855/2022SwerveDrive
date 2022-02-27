package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SwerveDriveSystem;

public class SwerveDriveMoveForward extends CommandBase {
    private final SwerveDriveSystem swerveDriveSystem;
    private double initialEncoderValue = 0;
    public SwerveDriveMoveForward(SwerveDriveSystem initialSwerveDriveSystem) {
        super(); //HOT
        swerveDriveSystem = initialSwerveDriveSystem;
        addRequirements(swerveDriveSystem);
    }

    @Override
    public void initialize() {
        initialEncoderValue = swerveDriveSystem.getEncoderFL();
    }

    @Override
    public void execute() {
        swerveDriveSystem.moveForward(100);
        //swerveDriveSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerveDriveSystem.getEncoderFL() - initialEncoderValue) > 100;
    }
    
}