
package frc.robot;  

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");	//creates the limelight table
	NetworkTableEntry x = limelightTable.getEntry("tx"); //the x "tx" offset from the limelight
	NetworkTableEntry y = limelightTable.getEntry("ty"); //the y "ty" offset from the limelight

	//creates getTargetX for getting and using the x "tx" offset from the limelight
	public double getTargetX() { 
		return x.getDouble(0);
	}

	//creates getTargetX for getting and using the x "tx" offset from the limelight
	public double getTargetY() { 
		return y.getDouble(0);
	}

}
