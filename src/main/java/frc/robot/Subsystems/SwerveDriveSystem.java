package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSystem extends SubsystemBase implements GenericDriveSystem{

    Wheel wheelFL = new Wheel(1, 2, 0, -0.758); //defines the front left wheel
    Wheel wheelBL = new Wheel(3, 4, 1, -0.454); //defines the back left wheel
    Wheel wheelBR = new Wheel(5, 6, 2, -0.143); //defines the back right wheel
    Wheel wheelFR = new Wheel(7, 8, 3, -0.077); //defines the front right wheel

    private void moveWheels(SwerveOutput swerve) {
        wheelFL.set(swerve.wheelAngles[0], swerve.wheelSpeeds[0]); //grabs information from the arrays and feeds it to the wheels 
        wheelFR.set(swerve.wheelAngles[1], swerve.wheelSpeeds[1]); //grabs information from the arrays and feeds it to the wheels 
        wheelBR.set(swerve.wheelAngles[2], swerve.wheelSpeeds[2]); //grabs information from the arrays and feeds it to the wheels 
        wheelBL.set(swerve.wheelAngles[3], swerve.wheelSpeeds[3]);
    }

    public double getEncoderFL() {
        return wheelFL.getDriveRelativeEncoderValue();
    }

    public double getEncoderBL() {
        return wheelBL.getDriveRelativeEncoderValue();
    }

    public double getEncoderBR() {
        return wheelBR.getDriveRelativeEncoderValue();
    }

    public double getEncoderFR() {
        return wheelFR.getDriveRelativeEncoderValue();
    }

    public void resetRelativeEncoders() {
      wheelFL.setRelativeEncoderToZero();
      wheelBL.setRelativeEncoderToZero();
      wheelBR.setRelativeEncoderToZero();
      wheelFR.setRelativeEncoderToZero();
    }
    public void moveManual(double x1, double y1, double x2, double theta_radians) {
        SwerveOutput swerve = Swerve.convertControllerToSwerve(x1, y1, x2, theta_radians);
        this.moveWheels(swerve);
    }

    @Override
    public void moveForward(double distance) {
        SwerveOutput swerve = Swerve.convertControllerToSwerve(0, -.5, 0, 0);
        this.moveWheels(swerve);
    }

    @Override
    public void moveBackward(double distance) {
        SwerveOutput swerve = Swerve.convertControllerToSwerve(0, 1, 0, 0);
        this.moveWheels(swerve);
    }

    @Override
    public void moveLeft(double distance) {
        SwerveOutput swerve = Swerve.convertControllerToSwerve(1, 0, 0, 0);
        this.moveWheels(swerve);
    }

    @Override
    public void moveRight(double distance) {
        SwerveOutput swerve = Swerve.convertControllerToSwerve(-1, 0, 0, 0);
        this.moveWheels(swerve);
    }

    @Override
    public void spinClockwise(double degrees) {
        SwerveOutput swerve = Swerve.convertControllerToSwerve(0, 0, -1, 0);
        this.moveWheels(swerve); 
    }

    @Override
    public void spinCounterclockwise(double degrees) {
        SwerveOutput swerve = Swerve.convertControllerToSwerve(0, 0, 1, 0);
        this.moveWheels(swerve);
    }

    @Override
    public void stop() {
        // SwerveOutput swerve = Swerve.convertControllerToSwerve(0, 0, 0, 0);
        // this.moveWheels(swerve);
        wheelFL.set(0,0); //grabs information from the arrays and feeds it to the wheels 
        wheelFR.set(0,0); //grabs information from the arrays and feeds it to the wheels 
        wheelBR.set(0,0); //grabs information from the arrays and feeds it to the wheels 
        wheelBL.set(0,0);
    }
    
}