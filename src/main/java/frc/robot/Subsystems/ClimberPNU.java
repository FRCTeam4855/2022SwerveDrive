package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ClimberPNU {

    private DoubleSolenoid climberArm = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);

    public void setClimberForward() {
        climberArm.set(Value.kForward);
    }

    public void setClimberReverse() {
        climberArm.set(Value.kReverse);
    }

    public boolean isClimberForward() {
        return climberArm.get() == Value.kForward;
    }

    public boolean isClimberReverse() {
        return climberArm.get() == Value.kReverse;
    }

}
