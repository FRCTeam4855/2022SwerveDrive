package frc.robot;

public class Constants {
    //Constants ++++++++++++
    public static final double JOYSTK_DZONE = .17; //global deadzone for all controllers
    public static final double LOW_GOAL_SPEED = 1200;
    public static final double HIGH_GOAL_SPEED = 4200;
    public static final double DRIVE_INIT_SPD = 0.75; //multiplier
    public static final double DRIVE_SLOW_SPD = 0.1; //multiplier

    // Conversions +++++++++
    public static final double RELATIVE_ENC_TO_FT = 7.4; //0.660715;//7.928591; //0.03666658; // converts relative encoder units on neo to inches

    //@@@@@@@@@
    //JOYSTICK to XBOX CONTROLLER KEY GUIDE (not code, just reference)
    // AXIS 0 = left stick X
    // AXIS 1 = left stick Y
    // AXIS 2 = left trigger 
    // AXIS 3 = right trigger
    // AXIS 4 = right stick X
    // AXIS 5 = right stick Y

    // BUTTON 1 = Xbox A
    // BUTTON 2 = Xbox B
    // BUTTON 3 = Xbox X
    // BUTTON 4 = Xbox Y
    //@@@@@@@@@@


    // Driver Mappings +++++
    public static final int GYRO_RESET = 4;
    public static final int ENCODER_RESET = 2;
    public static final int ORIENTATION_TOGGLE = 3; 
    public static final int DR_SPD_LIMITER = 6;



    // Operator Mappings +++
    public static final int CLIMBERARM_TOGGLE = 4;
    public static final int FLYWHEEL_TOGGLE = 1;
    public static final int INTAKEARM_TOGGLE = 2;
    


}
