
package frc.robot.Subsystems;  

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

	boolean lampOn = true;

    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");	//creates the limelight table
	NetworkTableEntry x = limelightTable.getEntry("tx"); //the x "tx" offset from the limelight
	NetworkTableEntry camMode = limelightTable.getEntry("camMode");
	NetworkTableEntry y = limelightTable.getEntry("ty"); //the y "ty" offset from the limelight
	NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");

	//creates getTargetX for getting and using the x "tx" offset from the limelight
	public double getTargetX() { 
		return x.getDouble(0);
	}

	//creates getTargetX for getting and using the x "tx" offset from the limelight
	public double getTargetY() { 
		return y.getDouble(0);
	}
	
	public void turnOnLamp() {
		camMode.setNumber(1);
		ledMode.setNumber(3);
		lampOn = true;
	}

	public void turnOffLamp() {
		ledMode.setNumber(0);
		camMode.setNumber(1);
		lampOn = false;
	}


	// public void toggleLamp() {
	// 	if(lampOn) {
	// 		turnOffLamp();
	// 	}else turnOnLamp();
	// }

	// public void toggleLamp() {
	//   if (lampOn2 == false) {
    //     limelight.turnOnLamp();
    //     lampOn2 = true;
    //   } else if (lampOn2 == true) {
    //     limelight.turnOffLamp();
    //     lampOn2 = false;
    //   }
	// 	}

}
