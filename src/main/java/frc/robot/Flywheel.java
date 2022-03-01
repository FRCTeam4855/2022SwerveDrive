package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel {
    private double velocitySetpoint = 0;        // the desired velocity setpoint for the flywheel
    private int speedUpTime = -1;               // the amount of program ticks remaining for the shooter to be fed by percentage input
    private boolean isRunning = false;          // whether or not the flywheel is running
    private final int MAX_SPEEDUP_TICKS = 50;   // the flywheel will have 50 ticks to increase the speed to roughly where it needs to be via percent output
    private final int ACCEPTABLE_ERROR = 70;    // the acceptable error to reasonably state that the shooter is ready to shoot
    private enum Phase {
        OFF, SPEED_UP, LOCK_IN
    }
    private Phase currentPhase = Phase.OFF;
    double kP = .00081;     // originally .00044
    double kI = 0;
    double kD = .0034;
    double kF = .000202;    // originally .000173

    // Define hardware
    private CANSparkMax flywheel; 
    private CANPIDController PID;
    private CANEncoder encoder;

    /**
     * Constructs the Shooter class.
     * @param sparkMaxId the ID of the CAN Spark Max that the flywheel runs off of
     * @param feederId the PWM ID of the feeder motor controller
     * @param pivotId the CAN ID of the Talon SRX that runs the pivot
     */
    public Flywheel(int sparkMaxId, int feederId, int pivotId) {
        flywheel = new CANSparkMax(sparkMaxId, MotorType.kBrushless);
        encoder = flywheel.getEncoder();
        PID = flywheel.getPIDController();
        PID.setOutputRange(-1, 1);
        PID.setP(kP);
        PID.setI(kI);
        PID.setD(kD);
        PID.setFF(kF);
    }

    /**
     * Sets the flywheel to a specified velocity. Returns false until the shooter has reached the acceptable speed
     * @param setpoint the desired velocity to set the flywheel to
     * @return true or false depending on the speed has been locked in
     */
    public boolean setFlywheelSpeed(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, 0, 4500);
        velocitySetpoint = setpoint;
        isRunning = true;
        switch (currentPhase) {
            case SPEED_UP:
                SmartDashboard.putString("Phase", "SPEED_UP");
                break;
            case OFF:
                SmartDashboard.putString("Phase", "OFF");
                break;
            case LOCK_IN:
                SmartDashboard.putString("Phase", "LOCK_IN");
                break;
            default:
                SmartDashboard.putString("Phase", "???");
                break;
        }
        if (currentPhase == Phase.OFF) {
            currentPhase = Phase.SPEED_UP;
            speedUpTime = MAX_SPEEDUP_TICKS;
        }
        // TODO getting consistent error messages: [CAN SPARK MAX] timed out while waiting for Periodic Status 1, Periodic Status 1
        if (speedUpTime > -1 && currentPhase == Phase.SPEED_UP) {
            double percentOutput = MathUtil.clamp(-((setpoint / 5100) + .015), 0, 1); // formerly (setpoint / 5100) - .025
            flywheel.set(percentOutput);
            speedUpTime --;
            if (speedUpTime <= 0) {
                speedUpTime = -1;
                currentPhase = Phase.LOCK_IN;
            }
        }
        if (currentPhase == Phase.LOCK_IN) {
            PID.setReference(-setpoint, ControlType.kVelocity);
            if (Math.abs(setpoint + encoder.getVelocity()) < ACCEPTABLE_ERROR) return true; else return false;
        }
        return false;
    }

    /**
     * Shuts off the flywheel. This should be used every time the shooter needs to stop shooting.
     */
    public void killFlywheel() {
        currentPhase = Phase.OFF;
        speedUpTime = -1;
        flywheel.set(0);
        isRunning = false;
        velocitySetpoint = 0;
    }

    /**
     * Sets the percentage output of the flywheel. Should be used conservatively.
     * @param speed the percent output of the speed
     */
    public void setFlywheelPercentSpeed(double speed) {
        if (!isRunning) flywheel.set(speed);
    }

    /**
     * Takes a distance from the lidar sensor and converts it to a velocity setpoint.
     * This was calculated based on the 2020 game and needs to be updated. Don't use!
     * @param dist the distance in inches
     * @return a double of the setpoint
     */
    public double getVelocityFromDistance(double dist) {
        if (dist < 90 || dist > 450) return getFlywheelVelocity(); // probably not valid, just use previous position
        return -0.008 * Math.pow(dist, 2) + 7.223 * dist + 1686;
    }

    /**
     * Gets the raw velocity reading from the flywheel encoder.
     * @return a double of the encoder velocity
     */
    public double getFlywheelVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Gets the velocity setpoint for the flywheel.
     * @return a double for the desired RPM of the flywheel
     */
    public double getFlywheelSetpoint() {
        return velocitySetpoint;
    }

    /**
     * Gets the voltage draw of the flywheel.
     * @return the voltage of the flywheel motor controller
     */
    public double getFlywheelCurrent() {
        return flywheel.getBusVoltage();
    }

    /**
     * Return whether or not the flywheel is running.
     * @return true if the flywheel is running, false if not
     */
    public boolean isRunning() {
        return isRunning;
    }

}