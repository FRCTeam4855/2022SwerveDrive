package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberMotor {

    public CANSparkMax armMotorL = new CANSparkMax(11, MotorType.kBrushless); // pid formerly 3
    public CANSparkMax armMotorR = new CANSparkMax(12, MotorType.kBrushless); // pid formerly 2

    public void climberUp() {
        armMotorL.set(.5);
        armMotorR.set(.5);
    }

    public void climberDown() {
        armMotorL.set(-.5);
        armMotorR.set(-.5);
    }

    public void climberStop() {
        armMotorL.set(0);
        armMotorR.set(0);
    }

    public void climberVariable(double input) {
        armMotorL.set(input);
        armMotorR.set(input);
    }
}
