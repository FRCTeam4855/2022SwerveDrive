package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeArmPneumatics {
    
    private DoubleSolenoid intakeArm = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 5);

    public void setIntakeArmDown() {
        intakeArm.set(Value.kForward);        
    }

    public void setIntakeArmUp() {
        intakeArm.set(Value.kReverse);
    }

    public boolean isIntakeArmDown() {
        return intakeArm.get() == Value.kForward;
    }

    public boolean isIntakeArmUp() {
        return intakeArm.get() == Value.kReverse;
    }
}
