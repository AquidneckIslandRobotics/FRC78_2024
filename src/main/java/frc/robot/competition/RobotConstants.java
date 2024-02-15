// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.competition;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.classes.ModuleConfig;
import frc.robot.classes.Structs.*;
import frc.robot.subsystems.Shooter.ShooterConfig;

/** This is the constants for the NEO */
class RobotConstants {
  public static final double WHEEL_WIDTH = 0.426; // Make sure this is from the wheel's center
  // of rotation
  public static final double WHEEL_DIAMETER =
      Units.inchesToMeters(4) * 1.274; // TODO measure more precisely

  public static final double ROBOT_RADIUS = Math.hypot(WHEEL_WIDTH / 2.0, WHEEL_WIDTH / 2.0);

  public static final int PIGEON_ID = 0;

  public static final String AT_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";

  public static final MotionLimits MOTION_LIMITS = new MotionLimits(5.6, 3 /*TODO */, 8, 12);

  public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG =
      new HolonomicPathFollowerConfig(
          new PIDConstants(2, 0.0, 0.0), // Translation PID constants
          new PIDConstants(2, 0.0, 0.0), // Rotation PID constants
          1, // Max module speed, in m/s
          RobotConstants.ROBOT_RADIUS, // Drive$ base radius in meters
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
  public static final FFConstants ROTATION_FF = new FFConstants(0.0, 0.0, 0.0);
  public static final double ORBITAL_FF_CONSTANT = 5;

  public static final RateLimits RATE_LIMITS = new RateLimits(11, 30);

  // WHEELS //
  public static final double DRIVE_GEAR_RATIO = (5.3571);
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

  public static final ModuleConfig MODULE_CONFIG =
      new ModuleConfig(
          new ClosedLoopParameters(0.1, 0, 0, 1 / RobotConstants.DRIVE_WHEEL_FREESPEED),
          new ClosedLoopParameters(18, 0, 0, 0),
          new FFConstants(0.201, 2.619, 0.634),
          RobotConstants.DRIVE_ENC_TO_METERS,
          RobotConstants.DRIVE_ENC_VEL_TO_METERS_PER_SECOND,
          RobotConstants.STEER_ENC_POS_TO_METERS,
          RobotConstants.STEER_ENC_VEL_TO_METERS,
          RobotConstants.DRIVE_INVERTED,
          RobotConstants.STEER_INVERTED,
          RobotConstants.STEER_ENC_INVERTED,
          RobotConstants.STEER_ENC_PID_MIN,
          RobotConstants.STEER_ENC_PID_MAX,
          RobotConstants.DRIVE_CURRENT_LIMIT,
          RobotConstants.STEER_CURRENT_LIMIT,
          RobotConstants.NOMINAL_VOLTAGE,
          RobotConstants.DRIVE_IDLE,
          RobotConstants.STEER_IDLE);

  // INTAKE
  public static final int INTAKE_TOP_ID = 10;
  public static final int INTAKE_BOTTOM_ID = 9;

  public static final double INTAKE_SPEED_IN = 0.75;

  public static final double INTAKE_SPEED_OUT = -0.5;

  // FEED //
  public static final int FEED_ID = 16;
  public static final int FEED_SENSOR_ID = 17;

  // Time of flight sensor range of interest
  public static final Range2D<Integer> TOF_RANGE = new Range2D<Integer>(10, 10, 11, 11);
  public static final double FEED_SENSOR_THRESHOLD = 125;

  public static final double FEED_INTAKE_SPEED = 0.15;
  public static final double FEED_OUTTAKE_SPEED = -0.5;
  public static final double FEED_FIRE_SPEED = 0.5;

  public static final ShooterConfig SHOOTER_CONFIG =
      new ShooterConfig(
          14,
          15,
          true,
          true,
          new Range(-1, 1),
          new Range(-1, 1),
          new PIDConstants(0, 0, 0),
          new PIDConstants(0, 0, 0),
          new FFConstants(0.16, 0.1065, 0.0, 0.0),
          new FFConstants(0.14, 0.1065, 0.0, 0.0));

  // Wrist Constants
  public static final int WRIST_ID = 13;

  public static final float WRIST_HIGH_LIM = 50;
  public static final float WRIST_LOW_LIM = 0;

  // CANDLE //
  public static final int CANDLE_ID = 1;

  // TODO auto stuff, but what for and is it needed?
  public static final double AUTO_SHOOT_SPEED = 500;
  public static final double AUTO_WRIST_SETPOINT = 0;
  public static final double WRIST_W2_TARGET = 34;
}
