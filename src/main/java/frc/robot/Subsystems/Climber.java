package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climber {
    private DoubleSolenoid climberArmL = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 0, 1);
    private DoubleSolenoid climberArmR = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 2, 3);
//     4 limit switches
//     public DoubleSolenoid getClimberArmL(){
//         return climberArmL;
//    }

    public void setClimberForward() {
        climberArmL.set(Value.kForward);
        climberArmR.set(Value.kForward);
    }

    public void setClimberReverse() {
        climberArmL.set(Value.kReverse);
        climberArmR.set(Value.kReverse);
    }

    public boolean isClimberForward() {
        return climberArmL.get() == Value.kForward && climberArmR.get() == Value.kForward;
    }

    public boolean isClimberReverse() {
        return climberArmL.get() == Value.kReverse && climberArmR.get() == Value.kReverse;
    }

}
