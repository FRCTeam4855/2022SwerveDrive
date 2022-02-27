package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeMotorSS extends SubsystemBase{
    private Spark intakeMotor = new Spark(0);
    
    public void forwardIntake() {
        intakeMotor.set(1);
    }

    public void reverseIntake() {
        intakeMotor.set(-1);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

}
