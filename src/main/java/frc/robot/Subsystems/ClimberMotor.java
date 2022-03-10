package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ClimberMotor {
    public Spark armMotorL = new Spark(3);
    public Spark armMotorR = new Spark(2);

    public void climberUP() {
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
