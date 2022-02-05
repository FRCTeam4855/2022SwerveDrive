/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;  

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the 
 * project. E
 */ 
public class Robot extends TimedRobot { 
  double theta_radians; //theta_radians is difference the angle the robot is at, and the zerod angle

  boolean driverOriented = false;

  private static final String kDefaultAuto = "Default"; //This is the first autonomous routine
  private static final String kCustomAuto = "My Auto"; //This is the second autonomous routine
  private String m_autoSelected; //This selects between the two autonomous
  private final SendableChooser<String> m_chooser = new SendableChooser<>(); 

  //CANEncoder encoder;

  //TODO Makes this an xbox controller
  Joystick joystick = new Joystick(0); //defines the controller 

  double startTime = -1;
  double elapsedTime = 0;

  Wheel wheelFL = new Wheel(1, 2, 0, -0.758); //defines the front left wheel
  Wheel wheelBL = new Wheel(3, 4, 1, -0.454); //defines the back left wheel
  Wheel wheelBR = new Wheel(5, 6, 2, -0.143); //defines the back right wheel
  Wheel wheelFR = new Wheel(7, 8, 3, -0.077); //defines the front right wheel
// Wheel Values: (Wheel Speed MC CAN ID, Angle MC CAN ID, Wheel Array Order?, wheel offset)
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto); //chooses the first auto program
    m_chooser.addOption("My Auto", kCustomAuto); //chooses the second auto program
    SmartDashboard.putData("Auto choices", m_chooser); //this does something

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
    SmartDashboard.putNumber("Encoder FL", wheelFL.absoluteEncoder.get()); //Displays Encoder Values
    SmartDashboard.putNumber("Encoder BL", wheelBL.absoluteEncoder.get()); //Displays Encoder Values
    SmartDashboard.putNumber("Encoder BR", wheelBR.absoluteEncoder.get()); //Displays Encoder Values
    SmartDashboard.putNumber("Encoder FR", wheelFR.absoluteEncoder.get()); //Displays Encoder Values
    
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
    /*double x1 = 0;
    double x2 = 0;
    double y1 = 0;
    theta_radians = 0;
    SwerveOutput swerve = Swerve.convertControllerToSwerve(x1, y1, x2, theta_radians);*/

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        
        break;
      case kDefaultAuto:
      default:
        
        // Put default auto code here
        if (startTime == -1) {
          startTime = System.nanoTime();
        }

        double elapsedTime = (System.nanoTime() - startTime) / 1000000000;
        SmartDashboard.putNumber("Elapsed Time right here", elapsedTime);

        double x1 = 0;
        double x2 = 0;
        double y1 = 0;
        //go forward
        if(elapsedTime >= 0.00 && elapsedTime <= 5.00) {
          y1 = -0.5;
        }
        //spin around 180
        if(elapsedTime >= 5.00 && elapsedTime <= 7.26){
          y1 = 0;
          x2 = 0.35;
        }
        //go forward again
        if(elapsedTime >= 8.26 && elapsedTime <= 13.26) {
          x2 = 0;
          y1 = -0.48;
        }
       

        SwerveOutput swerve = Swerve.convertControllerToSwerve(x1, y1, x2, theta_radians);
         wheelFL.set(swerve.wheelAngles[0], swerve.wheelSpeeds[0]); //grabs information from the arrays and feeds it to the wheels 
        wheelFR.set(swerve.wheelAngles[1], swerve.wheelSpeeds[1]); //grabs information from the arrays and feeds it to the wheels 
        wheelBR.set(swerve.wheelAngles[2], swerve.wheelSpeeds[2]); //grabs information from the arrays and feeds it to the wheels 
        wheelBL.set(swerve.wheelAngles[3], swerve.wheelSpeeds[3]); //grabs information from the arrays and feeds it to the wheels 
          

        
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double x1 = joystick.getRawAxis(0); //grabs the x-axis value of the left joystick
    double x2 = joystick.getRawAxis(4); //grabs the x-axis value of the right joystick
    double y1 = joystick.getRawAxis(1); //grabs the y-axis value of the left joystick

   //double theta_radians = gyro.getYaw()* Math.PI / 180;; //theta_radians is difference the angle the robot is at, and the zerod angle
    if (driverOriented) {
      theta_radians = gyro.getYaw()* Math.PI / 180;
    }else theta_radians = 0;
    SmartDashboard.putNumber("Gyro Get Raw", gyro.getYaw());
    SwerveOutput swerve = Swerve.convertControllerToSwerve(x1, y1, x2, theta_radians); //puts it all together?
    
    //SmartDashboard.putNumber("SetPoint BL", swerve.wheelAngles[3] - wheelBL.offSet0);
    //SmartDashboard.putNumber("Math", Math.sin(45));
    
    wheelFL.set(swerve.wheelAngles[0], swerve.wheelSpeeds[0]); //grabs information from the arrays and feeds it to the wheels 
    wheelFR.set(swerve.wheelAngles[1], swerve.wheelSpeeds[1]); //grabs information from the arrays and feeds it to the wheels 
    wheelBR.set(swerve.wheelAngles[2], swerve.wheelSpeeds[2]); //grabs information from the arrays and feeds it to the wheels 
    wheelBL.set(swerve.wheelAngles[3], swerve.wheelSpeeds[3]); //grabs information from the arrays and feeds it to the wheels 

    SmartDashboard.putNumber("AngleFL", swerve.wheelAngles[0]); //Displays the wheel angles on the smartdashboard
    SmartDashboard.putNumber("AngleFR", swerve.wheelAngles[1]); //Displays the wheel angles on the smartdashboard
    SmartDashboard.putNumber("AngleBR", swerve.wheelAngles[2]); //Displays the wheel angles on the smartdashboard
    SmartDashboard.putNumber("AngleBL", swerve.wheelAngles[3]); //Displays the wheel angles on the smartdashboard

    if (joystick.getRawButtonPressed(4)) { //zeros the gyro if you press the Y button
      gyro.reset();
    }
//This turns driver oriented on and off when x is pressed
    if (joystick.getRawButtonPressed(3)){
      if (driverOriented == false){
      driverOriented = true;
      }else{driverOriented = false;}
    }
    
   
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
