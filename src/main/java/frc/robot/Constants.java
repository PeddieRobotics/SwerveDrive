// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double JOYSTICK_THRESHOLD = 0.05; // placeholder value
    public static final double INCHES_TO_METERS = 0.0254; // Converting from inches to meters
    public static final double TRACKWIDTH_M = 0.53; // placeholder value
    public static final double WHEELBASE_M = 0.66; // placeholder value
    public static final double WHEELDIAMETER_IN = 4; // This is in inches
    public static final double WHEELRADIUS_M = (WHEELDIAMETER_IN / 2) * INCHES_TO_METERS; // Converting wheel radius to meters
    public static final double METERS_PER_SEC_TO_RPM = 30 / (Math.PI * WHEELRADIUS_M); // Converting Meters Per Second to Rotations Per Minute
    public static final double DRIVE_GEAR_RATIO = 8.16; // Gear ratio for the standard wheel (motor to wheel rotation: 8.16:1)
    public static final double ANGLE_GEAR_RATIO = 12.8; 

}
/*
 * Convert wheel radius from inches to meters: 2 * 0.0254 =
 * 0.0508
 * 
 * Convert target vel in m/s to rev/min (wheel): 3.704 * 30 / (pi * 0.0508) =
 * 696.272
 * 
 * Convert rev/min wheel to rev/min motor using gear ratio: 696.272 * 8.16 =
 * 5681.576 (YAY!!!!!)
 */