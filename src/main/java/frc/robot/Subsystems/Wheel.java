package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder; //CANEncoder

public class Wheel {

    CANSparkMax driveController; //defines the motor controller for the wheel speeds
    CANSparkMax steerController; //defines the motor cotrollers for the wheel angles
    public DutyCycleEncoder absoluteEncoder; //defines the encoder
    RelativeEncoder relativeEncoder; //defines a CAN encoder for the wheel //I don't think we need this

    public double offSet0; //this an offset that is later used

    boolean isFlipped = false; //whether the wheels are flipped or not
    double flipOffset = 0; //determines whether the wheel should go left or right when flipping
    double previousAngle = 0; //previous angle before the wheel was flipped
    
    PIDController pid = new PIDController(2, 0, 0); //sets up the PID loop, if you don't know what that is look it up

    private double getFlippedAngle() { //determines whether the wheel should be flipped or not
        if (isFlipped) {
            return .5;  //approximately the value of one rotation
        } else {
            return 0; //if the wheel should not be flipped, then this ensures it will not be flipped
        }
    }

    private void turnToAngle(double desiredAngle) {
        desiredAngle += flipOffset + getFlippedAngle() - offSet0; //sets the desired angle for and during the flipping of the wheels
        
        // If the wheel needs to turn more than 90 degrees to reach the target, flip the direction of the wheel
        double encoderSetpointDiff = Math.abs(absoluteEncoder.get() - desiredAngle); //defines the varible encoderSetpointDiff
        if (encoderSetpointDiff > .25 && encoderSetpointDiff < .75) {
            desiredAngle -= getFlippedAngle();
            isFlipped = !isFlipped;
            desiredAngle += getFlippedAngle();
        }

        if (previousAngle - desiredAngle > .5) { //.5 previously 185 
			flipOffset += 1; //1 previously 360
			desiredAngle += 1; //1 previously 360
        }
        if (previousAngle - desiredAngle < -.5) { //-.5 previously -185
            flipOffset -= 1; //1 previously 360
            desiredAngle -= 1; //1 previously 360
        }
        if (absoluteEncoder.get() - desiredAngle > 1) { //1 previously 380
            flipOffset += 1; //1 previously 360
            desiredAngle += 1; //1 previously 360
        }
        if (absoluteEncoder.get() - desiredAngle < -1) { //-1 previously -380
            flipOffset -= 1; //1 previously 360
            desiredAngle -= 1; //1 previously 360
        }
        previousAngle = desiredAngle; //states that the desiredAngle has been reach and is now the current/previous angle

        double desiredSpeed = pid.calculate(absoluteEncoder.get(), desiredAngle);
        SmartDashboard.putNumber("WheelSpeed", desiredSpeed); //displays the wanted speed on SmartDashboard
        steerController.set(MathUtil.clamp(desiredSpeed, -0.1, 0.1));
    }



    private void setSpeed(double motorSpeed) {
        if (isFlipped) motorSpeed *= -1; //this the speed the wheels will flip at, just don't change it
        driveController.set(motorSpeed * 0.5); //this is where you change the speed of the wheels
    }

    public void set(double setAngle, double speed) {
        turnToAngle(setAngle);
        setSpeed(speed);
    }

    public double getDriveRelativeEncoderValue(){
        return relativeEncoder.getPosition();
    }

    public void setRelativeEncoderToZero(){
        relativeEncoder.setPosition(0);
    }

    public Wheel(int driveControllerID, int steerControllerID, int absolutePort, double offSet1) {
        driveController = new CANSparkMax(driveControllerID, MotorType.kBrushless); //defining the motor controller for the wheel speeds and its port
        steerController = new CANSparkMax(steerControllerID, MotorType.kBrushless); //defining the motor controller for the wheel angles and its port
        absoluteEncoder = new DutyCycleEncoder(absolutePort); //defining the encoder and its port
        relativeEncoder = driveController.getEncoder();//relativePort); //maybe add another int to Wheel for this, would it just be 0 1 2 and 3 in Robot.java
        offSet0 = offSet1; //offSet for wheels
    }
}
