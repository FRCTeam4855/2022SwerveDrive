package frc.robot.Subsystems;

//import edu.wpi.first.wpilibj.motorcontrol.Spark;

//import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class PrettyLights {
    public final static double RAINBOW_RAINBOWPALETTE = -.99;
    public final static double RAINBOW_PARTYPALETTE = -.97;
    public final static double RAINBOW_OCEANPALETTE = -.95;
    public final static double RAINBOW_LAVAPALETTE = -.93;
    public final static double RAINBOW_FORESTPALETTE = -.91;
    public final static double RAINBOW_GLITTER = -.89;
    public final static double CONFETTI = -.87;
    public final static double SHOT_RED = -.85;
    public final static double SHOT_BLUE = -.83;
    public final static double SHOT_WHITE = -.81;
    public final static double SINELON_RAINBOWPALETTE = -.79;
    public final static double SINELON_PARTYPALETTE = -.77;
    public final static double SINELON_OCEANPALETTE = -.75;
    public final static double SINELON_LAVAPALETTE = -.73;
    public final static double SINELON_FORESTPALETTE = -.71;
    public final static double BPM_RAINBOWPALETTE = -.69;
    public final static double BPM_PARTYPALETTE = -.67;
    public final static double BPM_OCEANPALETTE = -.65;
    public final static double BPM_LAVAPALETTE = -.63;
    public final static double BPM_FORESTPALETTE = -.61;
    public final static double FIRE_MEDIUM = -.59;
    public final static double FIRE_LARGE = -.57;
    public final static double TWINKLES_RAINBOWPALETTE = -.55;
    public final static double TWINKLES_PARTYPALETTE = -.53;
    public final static double TWINKLES_OCEANPALETTE = -.55;
    public final static double TWINKLES_LAVAPALETTE = -.53;
    public final static double TWINKLES_FORESTPALETTE = -.51;
    public final static double COLORWAVES_RAINBOWPALETTE = -.45;
    public final static double COLORWAVES_PARTYPALETTE = -.43;
    public final static double COLORWAVES_OCEANPALETTE = -.41;
    public final static double COLORWAVES_LAVAPALETTE = -.39;
    public final static double COLORWAVES_FORESTPALETTE = -.37;
    public final static double LARSONSCAN_RED = -.35;
    public final static double LARSONSCAN_GRAY = -.33;
    public final static double LIGHTCHASE_RED = -.31;
    public final static double LIGHTCHASE_BLUE = -.29;
    public final static double LIGHTCHASE_GRAY = -.27;
    public final static double HEARTBEAT_RED = -.25;
    public final static double HEARTBEAT_BLUE = -.23;
    public final static double HEARTBEAT_WHITE = -.21;
    public final static double HEARTBEAT_GRAY = -.19;
    public final static double BREATH_RED = -.17;
    public final static double BREATH_BLUE = -.15;
    public final static double BREATH_GRAY = -.13;
    public final static double STROBE_RED = -.11;
    public final static double STROBE_BLUE = -.09;
    public final static double STROBE_GOLD = -.07;
    public final static double STROBE_WHITE = -.05;
    public final static double C1_END_TO_END_BLEND_TO_BLACK = -.03;
    public final static double C1_LARSONSCAN = -.01;
    public final static double C1_LIGHTCHASE = .01;
    public final static double C1_HEARTBEAT_SLOW = .03;
    public final static double C1_HEARTBEAT_MEDIUM = .05;
    public final static double C1_HEARTBEAT_FAST = .07;
    public final static double C1_BREATH_SLOW = .09;
    public final static double C1_BREATH_FAST = .11;
    public final static double C1_SHOT = .13;
    public final static double C1_STROBE = .15;
    public final static double C2_END_TO_END_BLEND_TO_BLACK = .17;
    public final static double C2_LARSONSCAN = .19;
    public final static double C2_LIGHTCHASE = .21;
    public final static double C2_HEARTBEAT_SLOW = .23;
    public final static double C2_HEARTBEAT_MEDIUM = .25;
    public final static double C2_HEARTBEAT_FAST = .27;
    public final static double C2_BREATH_SLOW = .29;
    public final static double C2_BREATH_FAST = .31;
    public final static double C2_SHOT = .33;
    public final static double C2_STROBE = .35;
    public final static double SPARKLE_C1_ON_C2 = .37;
    public final static double SPARKLE_C2_ON_C1 = .39;
    public final static double C1_AND_C2_GRADIENT = .41;
    public final static double C1_AND_C2_BPM = .43;
    public final static double C1_AND_C2_END_TO_END_BLEND = .45;
    public final static double END_TO_END_BLEND = .47; // ???
    public final static double C1_AND_C2_NO_BLEND = .49;
    public final static double C1_AND_C2_TWINKLES = .51;
    public final static double C1_AND_C2_COLOR_WAVES = .53;
    public final static double C1_AND_C2_SINELON = .55;
    public final static double HOT_PINK = .57;
    public final static double DARK_RED = .59;
    public final static double RED = .61;
    public final static double RED_ORANGE = .63;
    public final static double ORANGE = .65;
    public final static double GOLD = .67;
    public final static double YELLOW = .69;
    public final static double LAWN_GREEN = .71;
    public final static double LIME = .73;
    public final static double DARK_GREEN = .75;
    public final static double GREEN = .77;
    public final static double BLUE_GREEN = .79;
    public final static double AQUA = .81;
    public final static double SKY_BLUE = .83;
    public final static double DARK_BLUE = .85;
    public final static double BLUE = .87;
    public final static double BLUE_VIOLET = .89;
    public final static double VIOLET = .91;
    public final static double WHITE = .93;
    public final static double GRAY = .95;
    public final static double DARK_GRAY = .97;
    public final static double BLACK = .99;
    double pattern = C1_AND_C2_SINELON;

// C1 is purple and C2 is gold
    // double pattern = C1_AND_C2_SINELON;
    
    /**
     * Creates an instance of the Blinkin driver.
     * @param s the PWM port of the driver.
     */
    // public PrettyLights(int s) {
    //     leds = new Spark(s);
    // }

    /**
     * Sets the LEDs to a certain color. This function won't work when the robot is disabled.
     * @param color the color to set the lights to, coordinated with the constants set as static in the Blinkin class
     */
    // public void setLEDs(double color) {
    //     pattern = color;
    //     leds.set(color); 
    // }

    /**
     * Gets the current display pattern of the LEDs.
     * @return a number between -1 and 1 corresponding to the current pattern
     */
    // public double getLEDs() {
    //     return pattern;
    // }
    
}
