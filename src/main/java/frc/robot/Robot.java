/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.math.MathUtil;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
//import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the 
 * project. E
 */ 
public class Robot extends TimedRobot { 
 
  double theta_radians; //theta_radians is difference the angle the robot is at, and the zerod angle
  //double startTime = -1; //random value used for starting elapsedTime
  //double elapsedTime = 0; //is the elapsedTime for autonomous
  boolean driverOriented = false; //where the robot is in driver oriented or not
  double deviceIDFL = 1;

  private static final String kAuton1 = "Auton Mode 1"; //This is the first or default autonomous routine
  private static final String kAuton2 = "Auton Mode 2"; //This is the second autonomous routine
  private String m_autoSelected; //This selects between the two autonomous
  public final SendableChooser<String> m_chooser = new SendableChooser<>(); //creates the ability to switch between autons on SmartDashboard
  public boolean autozero = false;
  //driverController = new CANSparkMax(deviceIDFL, MotorType.kBrushless);
  //RelativeEncoder encoder;

  static Limelight limelight = new Limelight();

  //try using xbox controller instead
  Joystick joystick = new Joystick(0); //defines the driving controller 

  AHRS gyro = new AHRS(SPI.Port.kMXP); //defines the gyro

  Wheel wheelFL = new Wheel(1, 2, 0, -0.758); //defines the front left wheel
  Wheel wheelBL = new Wheel(3, 4, 1, -0.454); //defines the back left wheel
  Wheel wheelBR = new Wheel(5, 6, 2, -0.143); //defines the back right wheel
  Wheel wheelFR = new Wheel(7, 8, 3, -0.077); //defines the front right wheel
//Wheel Values: driveControllerID, steerControllerID, absolutePort(encoder), offSet1

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Auton Mode 1", kAuton1); //defines that this is the first or default auton
    m_chooser.addOption("Auton Mode 2", kAuton2); //defines that this is the second auton
    SmartDashboard.putData(m_chooser); //displays the auton options //maybe move to autonomousInit


  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder FL", wheelFL.absoluteEncoder.get()); //Displays Front Left Wheel Encoder Values
    SmartDashboard.putNumber("Encoder BL", wheelBL.absoluteEncoder.get()); //Displays Back Left Wheel Encoder Values
    SmartDashboard.putNumber("Encoder BR", wheelBR.absoluteEncoder.get()); //Displays Back Right Wheel Encoder Values
    SmartDashboard.putNumber("Encoder FR", wheelFR.absoluteEncoder.get()); //Displays Front Right Wheel Encoder Values
    SmartDashboard.putNumber("RelativeEncoder FL", wheelFL.getDriveRelativeEncoderValue());
    SmartDashboard.putNumber("RelativeEncoder BL", wheelBL.getDriveRelativeEncoderValue());
    SmartDashboard.putNumber("RelativeEncoder BR", wheelBR.getDriveRelativeEncoderValue());
    SmartDashboard.putNumber("RelativeEncoder FR", wheelFR.getDriveRelativeEncoderValue());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    m_autoSelected = m_chooser.getSelected(); //conects the auton options and the switch method where the autons are written
    SmartDashboard.putString("auton selected", m_autoSelected); //displays which auton is currently running
      wheelFL.setRelativeEncoderToZero();
      wheelBL.setRelativeEncoderToZero();
      wheelBR.setRelativeEncoderToZero();
      wheelFR.setRelativeEncoderToZero();

  }

  //goes in autonomousPeriodic when using elapsedTime based code
  //resets the startTime whenever an autonomous program is started
    /*if (startTime == -1) {
        startTime = System.nanoTime();
      }
    double elapsedTime = (System.nanoTime() - startTime) / 1000000000;*/ //sets elapsedTime

 /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Gyro Get Raw Auton", gyro.getYaw());
    //these values are inverted so negative and positive are reversed
      double x1 = 0; //defines left and right movement for auton
      double x2 = 0; //defines spinning movement for auton
      double y1 = 0; //defines forward and backward movement for auton

    switch (m_autoSelected) {
      //second auton code
      case kAuton2:
        x2 = -0.5;
        break; //end of second auton code

      //first or default auton code
      case kAuton1: 
      default: //is not a nescessaty, is like a fail safe and again states that this is the default auton
      
      if (autozero = false){
      //gyro.reset();
      autozero = true;
      }

      if (gyro.getYaw() > 55 && gyro.getYaw() < 150){
        x2 = 0;
      }else x2 = .3;

      break;
      }

      //is still a part of m_autoSelected and it grabs the things needed for driving in auton
        SwerveOutput swerve = Swerve.convertControllerToSwerve(x1, y1, x2, theta_radians); //grabs the driving variables and theata from Wheel.java
        wheelFL.set(swerve.wheelAngles[0], swerve.wheelSpeeds[0]); //grabs information from the arrays and feeds it to the wheels
        wheelFR.set(swerve.wheelAngles[1], swerve.wheelSpeeds[1]); //grabs information from the arrays and feeds it to the wheels
        wheelBR.set(swerve.wheelAngles[2], swerve.wheelSpeeds[2]); //grabs information from the arrays and feeds it to the wheels
        wheelBL.set(swerve.wheelAngles[3], swerve.wheelSpeeds[3]); //grabs information from the arrays and feeds it to the wheels

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("limelightX", limelight.getTargetX()); //displays the limelight X "tx" values on SmartDashboard
    SmartDashboard.putNumber("limelightY", limelight.getTargetY()); //displays the limelight Y "ty" values on SmartDashboard

    double x1 = joystick.getRawAxis(0); //connects the left and right drive movements to the drive controllers left x-axis
    double x2 = joystick.getRawAxis(4); //connects the spinning drive movements to the drive controllers right x-axis
    double y1 = joystick.getRawAxis(1); //connects the forwards and backwards drive movements to the drive controllers left y-axis

    //this tells the robot when it should be driverOriented or robotOriented
    if (driverOriented) {
      theta_radians = gyro.getYaw() * Math.PI / 180; //driverOriented
    }else theta_radians = 0; //robotOriented

    SmartDashboard.putNumber("Gyro Get Raw", gyro.getYaw()); //displays the gyro Yaw value on SmartDashboard
    SwerveOutput swerve = Swerve.convertControllerToSwerve(x1, y1, x2, theta_radians); //grabs the driving variables and theata from Wheel.java
    
    wheelFL.set(swerve.wheelAngles[0], swerve.wheelSpeeds[0]); //grabs information from the arrays and feeds it to the wheels 
    wheelFR.set(swerve.wheelAngles[1], swerve.wheelSpeeds[1]); //grabs information from the arrays and feeds it to the wheels 
    wheelBR.set(swerve.wheelAngles[2], swerve.wheelSpeeds[2]); //grabs information from the arrays and feeds it to the wheels 
    wheelBL.set(swerve.wheelAngles[3], swerve.wheelSpeeds[3]); //grabs information from the arrays and feeds it to the wheels 

    SmartDashboard.putNumber("AngleFL", swerve.wheelAngles[0]); //Displays the wheel angles on the smartdashboard
    SmartDashboard.putNumber("AngleFR", swerve.wheelAngles[1]); //Displays the wheel angles on the smartdashboard
    SmartDashboard.putNumber("AngleBR", swerve.wheelAngles[2]); //Displays the wheel angles on the smartdashboard
    SmartDashboard.putNumber("AngleBL", swerve.wheelAngles[3]); //Displays the wheel angles on the smartdashboard

    //zeros the gyro if you press the Y button
    if (joystick.getRawButtonPressed(4)) { 
      gyro.reset();
    }
  //This turns driver oriented on and off when x is pressed
    if (joystick.getRawButtonPressed(3)){
      if (driverOriented == false){
      driverOriented = true;
      }else{driverOriented = false;}
    }
    SmartDashboard.putBoolean("Driver Oriented", driverOriented);

    if (joystick.getRawButtonPressed(2)){//button B
      wheelFL.setRelativeEncoderToZero();
      wheelBL.setRelativeEncoderToZero();
      wheelBR.setRelativeEncoderToZero();
      wheelFR.setRelativeEncoderToZero();
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
  
}