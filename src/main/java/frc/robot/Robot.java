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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.SwerveDriveMoveForward;
import frc.robot.Commands.SwerveDriveStop;
import frc.robot.Flywheel.Phase;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.SwerveDriveSystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the 
 * project. E
 */ 
public class Robot extends TimedRobot { 
 
  double theta_radians; //theta_radians is difference the angle the robot is at, and the zerod angle
  boolean driverOriented = true; //where the robot is in driver oriented or not
  boolean climberForward = false;
  double deviceIDFL = 1;

  private static final String kAuton1 = "Auton Mode 1"; //This is the first or default autonomous routine
  private static final String kAuton2 = "Auton Mode 2"; //This is the second autonomous routine
  private String m_autoSelected; //This selects between the two autonomous
  public SendableChooser<String> m_chooser = new SendableChooser<>(); //creates the ability to switch between autons on SmartDashboard
  public boolean autozero = false;
  double autonsub1 = 0;
  double armMotorSpeedLimit = 1;
  double armMotorDeadZone = .02;
  double flywheelSpeed = .5;

  static Limelight limelight = new Limelight();
  Spark lights = new Spark(8); //this is not the right port

  Joystick joystick = new Joystick(0); //defines the driving controller
  Joystick operator = new Joystick(1);  

  Climber climber = new Climber();

  Spark armMotorL = new Spark(2);
  Spark armMotorR = new Spark(3);

  DoubleSolenoid intakeArmL = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 4, 5);
  DoubleSolenoid intakeArmR = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 6, 7);

  Spark intake = new Spark(0);
  Spark index = new Spark(1);
  Flywheel flywheel = new Flywheel(9);

  AHRS gyro = new AHRS(SPI.Port.kMXP); //defines the gyro

  // Wheel Values: driveControllerID, steerControllerID, absolutePort(encoder), offSet1
  SwerveDriveSystem driveSystem = new SwerveDriveSystem();

  




  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Don't use these here! Use them in autonomousInit or teleopInit
    //climber.setClimberForward();
    //intakeArmL.set(Value.kForward);
    //intakeArmR.set(Value.kForward);

    // CameraServer.startAutomaticCapture();
    // CvSink cvSink = CameraServer.getVideo();
    // CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

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
    SmartDashboard.putNumber("Encoder FL", driveSystem.wheelFL.getAbsoluteValue()); //Displays Front Left Wheel Encoder Values
    SmartDashboard.putNumber("Encoder BL", driveSystem.wheelBL.getAbsoluteValue()); //Displays Back Left Wheel Encoder Values
    SmartDashboard.putNumber("Encoder BR", driveSystem.wheelBR.getAbsoluteValue()); //Displays Back Right Wheel Encoder Values
    SmartDashboard.putNumber("Encoder FR", driveSystem.wheelFR.getAbsoluteValue()); //Displays Front Right Wheel Encoder Values
    SmartDashboard.putNumber("RelativeEncoder FL", driveSystem.getEncoderFL());
    SmartDashboard.putNumber("RelativeEncoder BL", driveSystem.getEncoderBL());
    SmartDashboard.putNumber("RelativeEncoder BR", driveSystem.getEncoderBR());
    SmartDashboard.putNumber("RelativeEncoder FR", driveSystem.getEncoderFR());

    SmartDashboard.putBoolean("Driver Oriented", driverOriented);
    SmartDashboard.putNumber("Gyro Get Raw", gyro.getYaw()); //pulls gyro values
    SmartDashboard.putNumber("Flywheel velocity", flywheel.getFlywheelVelocity());
  }


  @Override
  public void autonomousInit() {

    m_autoSelected = m_chooser.getSelected(); //conects the auton options and the switch method where the autons are written
    SmartDashboard.putString("auton selected", m_autoSelected); //displays which auton is currently running
    driveSystem.resetRelativeEncoders();
    gyro.reset();
    CommandScheduler.getInstance().setDefaultCommand(driveSystem, new SwerveDriveStop(driveSystem));
    SwerveDriveMoveForward swerveDriveMoveForward = new SwerveDriveMoveForward(driveSystem);
    swerveDriveMoveForward.schedule();
  }

  //goes in autonomousPeriodic when using elapsedTime based code
  //resets the startTime whenever an autonomous program is started
    /*if (startTime == -1) {
        startTime = System.nanoTime();
      }
    double elapsedTime = (System.nanoTime() - startTime) / 1000000000;*/ //sets elapsedTime




  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
   
    //these values are inverted so negative and positive are reversed
    double x1 = 0; //defines left and right movement for auton
    double x2 = 0; //defines spinning movement for auton
    double y1 = 0; //defines forward and backward movement for auton

    switch (m_autoSelected) {
      //second auton code
      case kAuton2:
        //x2 = -0.5;
        break; //end of second auton code

      //first or default auton code
      case kAuton1: 
      default: //is not a nescessaty, is like a fail safe and again states that this is the default auton
      
      // if ((gyro.getYaw() + 180) > 300 && autonsub1 == 0){
      //   x2 = 0;
      //   autonsub1 = 1;
      // }else x2 = .4;
      
      // if ((gyro.getYaw() + 180) > 307 && autonsub1 == 1){
      //   x2 = 0;
      //   autonsub1 = 2;
      // }else x2 = -.17;
      break;
      }

      //is still a part of m_autoSelected and it grabs the things needed for driving in auton
       // SwerveOutput swerve = Swerve.convertControllerToSwerve(x1, y1, x2, theta_radians); //grabs the driving variables and theata from Wheel.java
        // wheelFL.set(swerve.wheelAngles[0], swerve.wheelSpeeds[0]); //grabs information from the arrays and feeds it to the wheels
        // wheelFR.set(swerve.wheelAngles[1], swerve.wheelSpeeds[1]); //grabs information from the arrays and feeds it to the wheels
        // wheelBR.set(swerve.wheelAngles[2], swerve.wheelSpeeds[2]); //grabs information from the arrays and feeds it to the wheels
        // wheelBL.set(swerve.wheelAngles[3], swerve.wheelSpeeds[3]); //grabs information from the arrays and feeds it to the wheels

  }

  @Override
  public void teleopInit() {
    flywheel.killFlywheel();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //PrettyLights.AQUA;

    //flywheel.set(flywheelSpeed);

    SmartDashboard.putNumber("limelightX", limelight.getTargetX()); //displays the limelight X "tx" values on SmartDashboard
    SmartDashboard.putNumber("limelightY", limelight.getTargetY()); //displays the limelight Y "ty" values on SmartDashboard

    double x1 = -joystick.getRawAxis(0); //connects the left and right drive movements to the drive controllers left x-axis
    double x2 = -joystick.getRawAxis(4); //connects the spinning drive movements to the drive controllers right x-axis
    double y1 = -joystick.getRawAxis(1); //connects the forwards and backwards drive movements to the drive controllers left y-axis
    // if (Math.abs(y1) > .17) {
    //   driveSystem.moveForward(y1);
    // }

    //this tells the robot when it should be driverOriented or robotOriented
    if (driverOriented) {
      theta_radians = gyro.getYaw() * Math.PI / 180; //driverOriented
    }else theta_radians = 0; //robotOriented

    driveSystem.moveManual(x1, y1, x2, theta_radians);

    //zeros the gyro if you press the Y button
    // if (joystick.getRawButtonPressed(4)) { 
    if (joystick.getRawButtonPressed(Constants.GYRO_RESET)) { 
      gyro.reset();
    }

    //This turns driver oriented on and off when x is pressed
    if (joystick.getRawButtonPressed(3)){
      if (driverOriented == false){
      driverOriented = true;
      }else{driverOriented = false;}
    }

    //intake
    if (Math.abs(operator.getRawAxis(3)) > .15) {
      intake.set(-operator.getRawAxis(3) * .75);
    } else if (Math.abs(operator.getRawAxis(2)) > .15) {
      intake.set(operator.getRawAxis(2) * .75);
    } else intake.set(0);

    //intake arm pneumatics
    if (operator.getRawButtonPressed(2)){
      if (intakeArmL.get() == Value.kForward && intakeArmR.get() == Value.kForward) {
        intakeArmL.set(Value.kReverse);
        intakeArmR.set(Value.kReverse);
      } else if (intakeArmL.get() == Value.kReverse && intakeArmR.get() == Value.kReverse) {
        intakeArmL.set(Value.kForward);
        intakeArmR.set(Value.kForward);
      }
    }



    //climber stuff
    if (operator.getRawButtonPressed(Constants.CLIMBER_TOGGLE)){
      if (climber.isClimberForward()) {
        climber.setClimberReverse();
      } else if (climber.isClimberReverse()) {
        climber.setClimberForward();
      }
    }
    if (Math.abs(operator.getRawAxis(1)) < armMotorDeadZone) {
      armMotorL.set(0);
      armMotorR.set(0);
    }
    armMotorL.set(operator.getRawAxis(1));
    armMotorR.set(operator.getRawAxis(1));


    // Flywheel
    SmartDashboard.putString("flywheel state", flywheel.getCurrentPhase() == Phase.OFF ? "Off" : flywheel.getCurrentPhase() == Phase.SPEED_UP ? "Speed Up" : "Lock In");
    if (operator.getRawButtonPressed(1)) {
      if (flywheel.getCurrentPhase() == Phase.OFF) {
        flywheel.setFlywheelSpeed(4200);
      } else flywheel.killFlywheel();
     }

     if (flywheel.isRunning()) {
       if (flywheel.setFlywheelSpeed(flywheel.getFlywheelSetpoint())) {
         // We are OK to fire

       } else {
         // Yeah we're not OK to fire

       }
     }
     

    if (operator.getPOV() == 90){
      flywheel.setFlywheelSpeed(flywheel.getFlywheelSetpoint() + 1);
    }
    if (operator.getPOV() == 270){
      flywheel.setFlywheelSpeed(flywheel.getFlywheelSetpoint() - 1);
    }


    // Index
    index.set(Math.abs(operator.getRawAxis(5)) > .1 ? -operator.getRawAxis(5) * .5 : 0 );
  }

  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
  
}