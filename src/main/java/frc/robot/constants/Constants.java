// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

/** This is the constants file for all of the common constants */
public final class Constants {
  public static final double JOYSTICK_DEADBAND = 0.15;
  public static final double TRIGGER_DEADBAND = 0.1;

  public static final double UP_ADJUST = 0.5;
  public static final double DOWN_ADJUST = 0.25;

  public static final Translation2d BLUE_ORBIT_POSE = new Translation2d(0.5, 5.5);
  public static final Translation2d RED_ORBIT_POSE = new Translation2d(16, 5.5);
  public static final double ORBIT_RADIUS = 2;
  public static final double ORBIT_RADIUS_MARGIN = 1.0;
}
