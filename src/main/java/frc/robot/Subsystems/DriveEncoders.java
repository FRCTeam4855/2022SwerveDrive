package frc.robot.Subsystems;
import frc.robot.Constants;

public class DriveEncoders {
    SwerveDriveSystem driveSystemEncoders = new SwerveDriveSystem();

    //For getting the relative encoder values
    public double getEncoderFL() {
        return driveSystemEncoders.wheelFL.getDriveRelativeEncoderValue();
    }
    public double getEncoderBL() {
        return driveSystemEncoders.wheelBL.getDriveRelativeEncoderValue();
    }
    public double getEncoderBR() {
        return driveSystemEncoders.wheelBR.getDriveRelativeEncoderValue();
    }
    public double getEncoderFR() {
        return driveSystemEncoders.wheelFR.getDriveRelativeEncoderValue();
    }

    //reselts all relative encoders to zero
    public void resetRelativeEncoders() {
      driveSystemEncoders.wheelFL.setRelativeEncoderToZero();
      driveSystemEncoders.wheelBL.setRelativeEncoderToZero();
      driveSystemEncoders.wheelBR.setRelativeEncoderToZero();
      driveSystemEncoders.wheelFR.setRelativeEncoderToZero();
    }

    //relative encoder equation for auton
    public double getRelativeEncoderFT() {
        return Math.abs(driveSystemEncoders.wheelFL.getDriveRelativeEncoderValue()) / Constants.RELATIVE_ENC_TO_FT;
    }
}
