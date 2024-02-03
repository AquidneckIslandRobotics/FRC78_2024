// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.classes.Structs;

/** This is the constants for the NEO */
class RobotConstants {
  public static final double WHEEL_WIDTH =
      Units.inchesToMeters(18.75); // Make sure this is from the wheel's center of rotation
  public static final double WHEEL_DIAMETER =
      Units.inchesToMeters(4); // TODO measure more precisely

  public static final double ROBOT_RADIUS = Math.hypot(WHEEL_WIDTH / 2.0, WHEEL_WIDTH / 2.0);

  public static final int PIGEON_ID = 0;

  public static final String AT_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";

  public static final Structs.MotionLimits MOTION_LIMITS =
      new Structs.MotionLimits(4, 3 /*TODO */, 8, 18);

  public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG =
      new HolonomicPathFollowerConfig(
          new PIDConstants(5, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
          MOTION_LIMITS.maxSpeed, // Max module speed, in m/s
          ROBOT_RADIUS, // Drive base radius in meters
          new ReplanningConfig() // Default path replanning config.
          );
  // TODO Since the above and below are both PID constants for moving the robot to a target pose,
  // perhaps we could use just one set of constants for both Pathplanner and other drive commands?
  public static final PIDConstants TRANSLATION_PID = new PIDConstants(3.5, 0.0, 0.0);
  public static final PIDConstants ROTATION_PID = new PIDConstants(1, 0.0, 0.0);
  public static final Constraints ROTATION_CONSTRAINTS =
      new Constraints(MOTION_LIMITS.maxAngularSpeed, MOTION_LIMITS.maxAngularAcceleration);
  // TODO
  public static final Structs.FFConstants ROTATION_FF =
      new Structs.FFConstants(
          0.40076 / (ROBOT_RADIUS * 2 * Math.PI),
          2.3621 / (ROBOT_RADIUS * 2 * Math.PI),
          0.41229 / (ROBOT_RADIUS * 2 * Math.PI));

  public static final Structs.RateLimits RATE_LIMITS = new Structs.RateLimits(11, 30);

  // WHEELS
  public static final double DRIVE_GEAR_RATIO = (6.75);
  public static final double DRIVE_MOTOR_FREESPEED_RPS = 5676 / 60; // Free RPM of NEO to RPS
  public static final double DRIVE_WHEEL_FREESPEED =
      (DRIVE_MOTOR_FREESPEED_RPS * (WHEEL_DIAMETER * Math.PI))
          / DRIVE_GEAR_RATIO; // Converted for wheel

  public static final double DRIVE_ENC_TO_METERS = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
  public static final double DRIVE_ENC_VEL_TO_METERS_PER_SECOND =
      ((WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO) / 60;
  public static final boolean STEER_ENC_INVERTED = true;
  public static final boolean DRIVE_INVERTED = true;
  public static final boolean STEER_INVERTED = true;

  public static final double STEER_ENC_POS_TO_METERS =
      1; // factor of steer encoder to meters(conversion factor)
  public static final double STEER_ENC_VEL_TO_METERS = 1 / 60; // factor of vel to meters

  public static final int DRIVE_CURRENT_LIMIT = 50; // amps
  public static final int STEER_CURRENT_LIMIT = 20; // amps

  public static final double NOMINAL_VOLTAGE = 12;

  public static final IdleMode DRIVE_IDLE = IdleMode.kBrake;
  public static final IdleMode STEER_IDLE = IdleMode.kCoast;

  public static final double STEER_ENC_PID_MIN = 0.0;
  public static final double STEER_ENC_PID_MAX = STEER_ENC_POS_TO_METERS; // TODO
}
