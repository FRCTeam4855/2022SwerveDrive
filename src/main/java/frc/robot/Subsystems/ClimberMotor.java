package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder; //CANEncoder
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class ClimberMotor {
    //public Spark armMotorL = new Spark(3);
    //public Spark armMotorR = new Spark(2);

    public CANSparkMax armMotorL = new CANSparkMax(11, MotorType.kBrushless); // pid formerly 3
    public CANSparkMax armMotorR = new CANSparkMax(12, MotorType.kBrushless); // pid formerly 2
    //public RelativeEncoder encoderLeft = armMotorL.getEncoder();
    //public RelativeEncoder encoderRight = armMotorR.getEncoder();

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
        //if (Constants.CLIMB_ENCODER_LIMIT_LEFT < encoderLeft.getPosition() && input > 0) armMotorL.set(input);
        //if (Constants.CLIMB_ENCODER_LIMIT_RIGHT < encoderRight.getPosition() && input > 0) armMotorR.set(input);
        armMotorL.set(input);
        armMotorR.set(input);
    }
}
