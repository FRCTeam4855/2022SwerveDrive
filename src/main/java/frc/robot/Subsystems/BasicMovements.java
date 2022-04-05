package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Subsystems.SwerveDriveSystem;

public class BasicMovements extends SubsystemBase implements GenericDriveSystem{

    SwerveDriveSystem driveValues = new SwerveDriveSystem();
    
    double y1 = 0;
    double x1 = 0;
    double x2 = 0;

    @Override
    public void moveForward(double distance) {
        driveValues.moveManual(x1, y1, x2, 0);
        if (distance > driveValues.getRelativeEncoderFT()) {
            y1 = 0;
        } else { // when less than or equal to run
            y1 = .4;
        }
    }

    @Override
    public void moveBackward(double distance) {
        
    }

    @Override
    public void moveLeft(double distance) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void moveRight(double distance) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void spinClockwise(double degrees) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void spinCounterclockwise(double degrees) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }
    
}
