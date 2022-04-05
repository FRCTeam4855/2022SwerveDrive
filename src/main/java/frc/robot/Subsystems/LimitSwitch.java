package frc.robot.Subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;

public class LimitSwitch {
    
    private I2C.Port myPort;
    private ColorSensorV3 colorSensor;
    
    public LimitSwitch(I2C.Port port){
        this.myPort = port;
        this.colorSensor = new ColorSensorV3(myPort);
    }
    public double getProximity(){
        return this.colorSensor.getProximity();
    }
}
