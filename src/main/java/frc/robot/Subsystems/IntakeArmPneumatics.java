package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeArmPneumatics {
    private DoubleSolenoid intakeArm = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 4, 5);

    public void setIntakeArmDown() {
        intakeArm.set(Value.kForward);
        //climberArm.set(Value.kForward);
    }

    public void setIntakeArmUp() {
        intakeArm.set(Value.kReverse);
        //climberArm.set(Value.kReverse);
    }

    public boolean isIntakeArmDown() {
        return intakeArm.get() == Value.kForward;
        //return climberArm.get() == Value.kForward;
    }

    public boolean isIntakeArmUp() {
        return intakeArm.get() == Value.kReverse;
        //return climberArm.get() == Value.kReverse;
    }
}
