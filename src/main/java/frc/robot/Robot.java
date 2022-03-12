/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.ClimberMotor;
import frc.robot.Subsystems.ClimberPNU;
import frc.robot.Subsystems.Flywheel;
import static frc.robot.Constants.*;
import frc.robot.Subsystems.IntakeArmPneumatics;
import frc.robot.Subsystems.PrettyLights;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Subsystems.Wheel;
import frc.robot.Subsystems.Flywheel.Phase;
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

  private static final String kAuton1 = "Auton Mode 1"; //This is the first or default autonomous routine
  private static final String kAuton2 = "Auton Mode 2"; //Drives forward, picks up a ball, and fires both
  private static final String kAuton3 = "Auton Mode 3"; //Drives forward and stops

  private String m_autoSelected; //This selects between the two autonomous
  public SendableChooser<String> m_chooser = new SendableChooser<>(); //creates the ability to switch between autons on SmartDashboard

  static Limelight limelight = new Limelight();

  Joystick joystick = new Joystick(0);
  Joystick operator = new Joystick(1); 

  ClimberPNU climberArms = new ClimberPNU();
  ClimberMotor climberMotors = new ClimberMotor();
  IntakeArmPneumatics  intakeArm = new IntakeArmPneumatics();
  Spark intake = new Spark(0);
  Spark index = new Spark(1);
  Flywheel flywheel = new Flywheel(9);

  UsbCamera frontCamera;
  UsbCamera climbCamera;

  public Spark leds = new Spark(8);
  double pattern = PrettyLights.C1_STROBE;
    public void setLEDs(double color) {
        pattern = color;
        leds.set(color); 
    }


  AHRS gyro = new AHRS(SPI.Port.kMXP); //defines the gyro

  SwerveDriveSystem driveSystem = new SwerveDriveSystem();

  // Autonomous kinematics/path-following
  // SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
  //   new Translation2d(-11.5, 11.5),
  //   new Translation2d(-11.5, -11.5),
  //   new Translation2d(-11.5, 11.5),
  //   new Translation2d(11.5, -11.5)
  // );
  // HolonomicDriveController holonomicController = new HolonomicDriveController(
  //   new PIDController(1, 0, 0), 
  //   new PIDController(1, 0, 0),
  //   new ProfiledPIDController(1, 0, 0, 
  //     new TrapezoidProfile.Constraints(6.28, 3.14)
  //   )
  // );
  
  double autox1 = 0; //defines left and right movement for auton
  double autox2 = 0; //defines spinning movement for auton
  double autoy1 = 0; //defines forward and backward movement for auton



  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    frontCamera = CameraServer.startAutomaticCapture();
    frontCamera.setResolution(240, 180);
    frontCamera.setFPS(8);
    
    climbCamera = CameraServer.startAutomaticCapture();
    climbCamera.setResolution(240, 180);
    climbCamera.setFPS(8);

    intakeArm.setIntakeArmDown();
    climberArms.setClimberReverse();

    m_chooser.setDefaultOption("Auton Mode 1", kAuton1);
    m_chooser.addOption("Auton Mode 2", kAuton2);
    m_chooser.addOption("Auton Mode 3", kAuton3);
    //m_chooser.addOption("Auton Mode 4", kAuton4);
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
    SmartDashboard.putNumber("DriveEncoder FL", driveSystem.getEncoderFL());

    SmartDashboard.putNumber("limelightX", limelight.getTargetX()); //displays the limelight X "tx" values on SmartDashboard
    SmartDashboard.putNumber("limelightY", limelight.getTargetY()); //displays the limelight Y "ty" values on SmartDashboard

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
    // CommandScheduler.getInstance().setDefaultCommand(driveSystem, new SwerveDriveStop(driveSystem));
    // SwerveDriveMoveForward swerveDriveMoveForward = new SwerveDriveMoveForward(driveSystem);
    // swerveDriveMoveForward.schedule();
  }


  @Override
  public void autonomousPeriodic() {
    //CommandScheduler.getInstance().run();
   
    //these values are inverted so negative and positive are reversed

    switch (m_autoSelected) {
      //first or default auton code
      // drive forward
      case kAuton1: 
      default: 
      if (driveSystem.getRelativeEncoderFT() < 10) {//74 is 10 ft
        autoy1 = .4;
      } else {
        autoy1 = 0;
      }
      break;

      // Drive forward, fire cargo
      case kAuton2:
      flywheel.setFlywheelSpeed(4500);
        if (driveSystem.getRelativeEncoderFT() < 12) {
          autoy1 = .4;
          intake.set(.75);
        } else {
          autoy1 = 0;
          index.set(.5);
          // if (flywheel.setFlywheelSpeed(4500)) {
            // Fire cargoaw 
            // index.set(-.5);
          //}
        
        }
        break;

      // Drive forward, pick up a cargo, fire both
      case kAuton3:
      intake.set(.75);
      if (driveSystem.getRelativeEncoderFT() < 4) {
        autoy1 = .4;
        intake.set(.75);
      } else {
        autoy1 = 0;
        if (flywheel.setFlywheelSpeed(3400)) {
          // Fire cargo
          index.set(-.5);
        }
      }

        break;
      
      }

      driveSystem.moveManual(autox1, autoy1, autox2, 0);
  }

  @Override
  public void teleopInit() {
    flywheel.killFlywheel();
    setLEDs(PrettyLights.BPM_RAINBOWPALETTE);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double x1 = -joystick.getRawAxis(0); //connects the left and right drive movements to the drive controllers left x-axis
    double x2 = -joystick.getRawAxis(4); //connects the spinning drive movements to the drive controllers right x-axis
    double y1 = -joystick.getRawAxis(1); //connects the forwards and backwards drive movements to the drive controllers left y-axis

    //Driver Controller
    //this tells the robot when it should be driverOriented or robotOriented
    if (driverOriented) {
      theta_radians = gyro.getYaw() * Math.PI / 180; //driverOriented
    } else theta_radians = 0; //robotOriented


    // Drive the robot
    Wheel.SpeedSetting driveSpeed = Wheel.SpeedSetting.NORMAL;
    if (joystick.getRawButton(6)) driveSpeed = Wheel.SpeedSetting.TURBO;
    if (joystick.getRawAxis(2) > .5) driveSpeed = Wheel.SpeedSetting.PRECISE;
    driveSystem.moveManual(x1, y1, x2, theta_radians, driveSpeed);
    
    // Reset the relative encoders
    if (joystick.getRawButtonPressed(ENCODER_RESET)) {
      driveSystem.resetRelativeEncoders();
    }

    //zeros the gyro if you press the Y button
    if (joystick.getRawButtonPressed(GYRO_RESET)) { 
      gyro.reset();
      setLEDs(PrettyLights.WHITE);
    }

    //This turns driver oriented on or off when x is pressed
    if (joystick.getRawButtonPressed(ORIENTATION_TOGGLE)){
      driverOriented = !driverOriented;
    }




    //Operator Controller
    //intake
    if (Math.abs(operator.getRawAxis(3)) > JOYSTK_DZONE) {
      intake.set(-operator.getRawAxis(3) * .75);
    } else if (Math.abs(operator.getRawAxis(2)) > JOYSTK_DZONE) {
      intake.set(operator.getRawAxis(2) * .75);
    } else intake.set(0);

    //intake arm pneumatics
    if (operator.getRawButtonPressed(INTAKEARM_TOGGLE)){
      if (intakeArm.isIntakeArmDown()) {
        intakeArm.setIntakeArmUp();
      } else if (intakeArm.isIntakeArmUp()) {
        intakeArm.setIntakeArmDown();;
      }
    }

    //climber stuff
    if (operator.getRawButtonPressed(CLIMBERARM_TOGGLE)){
      if (climberArms.isClimberForward()) {
        setLEDs(PrettyLights.BPM_RAINBOWPALETTE);
        climberArms.setClimberReverse();
      } else if (climberArms.isClimberReverse()) {
        climberArms.setClimberForward();
      }
    }
    if (Math.abs(operator.getRawAxis(1)) < JOYSTK_DZONE) {
      climberMotors.climberStop();
    } else {
      climberMotors.climberVariable(operator.getRawAxis(1));
    }


    // Flywheel
    SmartDashboard.putBoolean("flywheel state", flywheel.getCurrentPhase() == Phase.LOCK_IN);
    if (operator.getRawButtonPressed(FLYWHEEL_TOGGLE) || operator.getRawButtonPressed(3)) {
      // Turn ON the flywheel
      if (flywheel.getCurrentPhase() == Phase.OFF) {
        if (operator.getRawButton(3)) {
          // Low goal
          flywheel.setFlywheelSpeed(LOW_GOAL_SPEED);
        } else {
          // High goal
          flywheel.setFlywheelSpeed(HIGH_GOAL_SPEED);
        }
      } else flywheel.killFlywheel();
     }

     if (flywheel.isRunning()) {
       if (flywheel.setFlywheelSpeed(flywheel.getFlywheelSetpoint())) {
         // We are OK to fire
         setLEDs(PrettyLights.BREATH_BLUE);
       } else {
         // Yeah we're not OK to fire
        setLEDs(PrettyLights.BREATH_RED);
       }
     }
     

    if (operator.getPOV() == 90){
      flywheel.setFlywheelSpeed(flywheel.getFlywheelSetpoint() + 10);
    }
    if (operator.getPOV() == 270){
      flywheel.setFlywheelSpeed(flywheel.getFlywheelSetpoint() - 10);
    }

    // Index
    index.set(Math.abs(operator.getRawAxis(5)) > .15 ? (-(Math.abs(operator.getRawAxis(5)) / operator.getRawAxis(5) * .5)): 0 );
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
  
}