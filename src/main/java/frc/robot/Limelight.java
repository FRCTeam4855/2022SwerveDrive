package frc.robot;  

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");	// creates the limelight table
	NetworkTableEntry x = limelightTable.getEntry("tx");									// the x offset from the crosshairs
	NetworkTableEntry y = limelightTable.getEntry("ty");

  /**
	 * Returns the x value of the target relative to Limelight's crosshairs
	 * @return a double in Limelight units the displacement between the target and the crosshairs
	 */
	public double getTargetX() {
		return x.getDouble(0);
	}

	/**
	 * Returns the y value of the target relative to Limelight's crosshairs
	 * @return a double in Limelight units the displacement between the target and the crosshairs
	 */
	public double getTargetY() {
		return y.getDouble(0);
	}
}
