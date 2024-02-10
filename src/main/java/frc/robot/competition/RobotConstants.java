// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.classes.Structs;

/** This is the constants for the NEO */
class RobotConstants {
  public static final double WHEEL_WIDTH =
      Units.inchesToMeters(18.75); // Make sure this is from the wheel's center
  // of rotation
  public static final double WHEEL_DIAMETER =
      Units.inchesToMeters(4); // TODO measure more precisely

  public static final double ROBOT_RADIUS = Math.hypot(WHEEL_WIDTH / 2.0, WHEEL_WIDTH / 2.0);

  public static final int PIGEON_ID = 0;

  public static final String AT_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";
  public static final Transform3d CAM1_OFFSET =
      new Transform3d(
          new Translation3d(0.31, 0.0, 0.15), new Rotation3d(0, Math.toRadians(-15), 0));

  public static final Structs.MotionLimits MOTION_LIMITS =
      new Structs.MotionLimits(4, 3 /*TODO */, 8, 12);

  public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG =
      new HolonomicPathFollowerConfig(
          new PIDConstants(5, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
          RobotConstants.MOTION_LIMITS.maxSpeed, // Max module speed, in m/s
          RobotConstants.ROBOT_RADIUS, // Drive base radius in meters
          new ReplanningConfig() // Default path replanning config.
          );
  // TODO Since the above and below are both PID constants for moving the robot to
  // a target pose, perhaps we could use just one set of constants for both
  // Pathplanner and other drive commands?
  public static final PIDConstants TRANSLATION_PID = new PIDConstants(3.5, 0.0, 0.0);
  public static final PIDConstants ROTATION_PID = new PIDConstants(3.5, 0.0, 0.0);
  public static final Constraints ROTATION_CONSTRAINTS =
      new Constraints(MOTION_LIMITS.maxAngularSpeed, MOTION_LIMITS.maxAngularAcceleration);
  // TODO
  public static final Structs.FFConstants ROTATION_FF = new Structs.FFConstants(0.0, 0.0, 0.0);
  public static final double ORBITAL_FF_CONSTANT = 5;

  public static final Structs.RateLimits RATE_LIMITS = new Structs.RateLimits(11, 30);

  // WHEELS
  public static final double DRIVE_GEAR_RATIO = (6.75); // TODO need to update for L3
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

  public static final double STEER_ENC_VEL_TO_METERS = 1.0 / 60.0; // factor of vel to meters

  public static final int DRIVE_CURRENT_LIMIT = 50; // amps
  public static final int STEER_CURRENT_LIMIT = 20; // amps

  public static final double NOMINAL_VOLTAGE = 12;

  public static final IdleMode DRIVE_IDLE = IdleMode.kBrake;
  public static final IdleMode STEER_IDLE = IdleMode.kCoast;

  public static final double STEER_ENC_PID_MIN = 0.0;
  public static final double STEER_ENC_PID_MAX = STEER_ENC_POS_TO_METERS; // TODO

  // INTAKE
  public static final int INTAKE_TOP_ID = 10;
  public static final int INTAKE_BOTTOM_ID = 9;

  public static final double INTAKE_SPEED_IN = 0.75;

  public static final double INTAKE_SPEED_OUT = -0.5;

  // SHOOTER

  public static final int FLYWHEEL_TOP_ID = 14;
  public static final int FLYWHEEL_BOTTOM_ID = 15;
  public static final int FEED_ID = 0; // TODO
  public static final int BELT_ID = 0; // TODO

  // Constants - TOP FLYWHEEL
  public static final double FLYWHEEL_TOP_MIN = -1;
  public static final double FLYWHEEL_TOP_MAX = 1;
  // PID Consants - TOP FLYWHEEL
  public static final double FLYWHEEL_TOP_P = 0.11; // TODO
  public static final double FLYWHEEL_TOP_I = 0; // TODO
  public static final double FLYWHEEL_TOP_D = 0; // TODO

  public static final double FLYWHEEL_TOP_S = 0.5; // TODO
  public static final double FLYWHEEL_TOP_V = 0.12; // TODO
  public static final double FLYWHEEL_TOP_FF = 0.00015673981;

  // Constants - BOTTOM FLYWHEEL
  public static final double FLYWHEEL_BOTTOM_MIN = -1;
  public static final double FLYWHEEL_BOTTOM_MAX = 1;
  // PID Constants - BOTTOM FLYWHEEL
  public static final double FLYWHEEL_BOTTOM_P = 0.11; // TODO
  public static final double FLYWHEEL_BOTTOM_I = 0; // TODO
  public static final double FLYWHEEL_BOTTOM_D = 0; // TODO

  public static final double FLYWHEEL_BOTTOM_S = 0.5; // TODO
  public static final double FLYWHEEL_BOTTOM_V = 0.12; // TODO
  public static final double FLYWHEEL_BOTTOM_FF = 0.00015673981;

  /** Hood's angle of elevation in degrees */
  public static final double HOOD_ANGLE = 45.0; // TODO

  // Wrist Constants
  public static final int WRIST_ID = 13;

  public static final float WRIST_HIGH_LIM = 139; // 139
  public static final float WRIST_LOW_LIM = 100; // 90

  public static final double AUTO_SHOOT_SPEED = 500;
  public static final double AUTO_WRIST_SETPOINT = 0;
}
