package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Climber {
    DoubleSolenoid climerArmL = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 0, 1);
    DoubleSolenoid climerArmR = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 2, 3);
    Spark armMotorL = new Spark(2);
    Spark armMotorR = new Spark(3);
    //4 limit switches

}
//DoubleSolenoid intakeL = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 4, 5);
//DoubleSolenoid intakeR = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 6, 7);