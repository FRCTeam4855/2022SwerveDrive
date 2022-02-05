package frc.robot;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;//CANEncoder

public class Wheel {
    CANSparkMax driveController; //defines the motor controller for the wheel speed
    CANSparkMax steerController; //defines the motor cotrollers for the wheel angle
    RelativeEncoder encoder; //defines a CAN encoder for the wheel
    DutyCycleEncoder absoluteEncoder; 
    public double offSet0; //

    // This is for wheel flipping
    boolean isFlipped = false; //holds a true or false value if the wheels are flipped or not
    double flipOffset = 0; //determines whether if the wheel should go left or right when flipping
    double previousAngle = 0; //previous angle before the wheel was flipped
    
    PIDController pid = new PIDController(2, 0, 0); //sets up the PID loop, if you don't know what that is look it up

    private double getFlippedAngle() { //determines whether if the wheel should be flipped or not
        if (isFlipped) {
            return .5;  // approximately the value of one rotation
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
        previousAngle = desiredAngle;

        double desiredSpeed = pid.calculate(absoluteEncoder.get(), desiredAngle);
        SmartDashboard.putNumber("WheelSpeed", desiredSpeed);
        steerController.set(MathUtil.clamp(desiredSpeed, -0.1, 0.1));
    }

    private void setSpeed(double motorSpeed) {
        if (isFlipped) motorSpeed *= -1; 
        driveController.set(motorSpeed * 0.5); // this is where you change the speed
    }

    public void set(double setAngle, double speed) {
        turnToAngle(setAngle);
        setSpeed(speed);
    }

    public Wheel(int driveControllerID, int steerControllerID, int absolutePort, double offSet1) {
        driveController = new CANSparkMax(driveControllerID, MotorType.kBrushless); //defining the motor controller for the wheel speeds
        steerController = new CANSparkMax(steerControllerID, MotorType.kBrushless); //defining the motor controller for the wheel angles
        //encoder = new RelativeEncoder(steerController); //defining the encoder that finds the wheel angle //This is an error. Is it important?
        absoluteEncoder = new DutyCycleEncoder(absolutePort);
        offSet0 = offSet1; //offSet for wheels
    }
}
