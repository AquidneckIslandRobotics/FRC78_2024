// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.robot.Classes.ModuleConfig;

/** This is the constants for the NEO */
public class RobotConstants {

  // #region Test Chassis

  //   public static final double WHEEL_WIDTH =
  //       Units.inchesToMeters(18.75); // Make sure this is from the wheel's center of rotation
  //   public static final double WHEEL_DIAMETER =
  //       Units.inchesToMeters(4); // TODO measure more precisely

  //   // public static final  WHEEL_POSITIONS =  TODO

  //   public static final ModuleConfig[] MOD_CONFIGS = {
  //     new ModuleConfig(1, 2, 9, 0), // LU
  //     new ModuleConfig(3, 4, 10, 0), // RU
  //     new ModuleConfig(5, 6, 11, 0), // LD
  //     new ModuleConfig(7, 8, 12, 0) // RD
  //   };

  //   public static final int PIGEON_ID = 0;

  //   public static final double MAX_SPEED = 4; // TODO
  //   public static final double MAX_ANGULAR_VELOCITY = 8; // TODO set temporarily, to look into
  // later

  //   // WHEELS

  //   public static final double DRIVE_GEAR_RATIO = (6.75);

  //   public static final double DRIVE_MOTOR_FREESPEED_RPS = 5676 / 60; // Free RPM of NEO to RPS
  //   public static final double DRIVE_WHEEL_FREESPEED =
  //       (DRIVE_MOTOR_FREESPEED_RPS * (WHEEL_DIAMETER * Math.PI))
  //           / DRIVE_GEAR_RATIO; // Converted for wheel

  //   public static final double DRIVE_ENC_TO_METERS = (WHEEL_DIAMETER * Math.PI) /
  // DRIVE_GEAR_RATIO;
  //   public static final double DRIVE_ENC_VEL_TO_METERS =
  //       ((WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO) / 60;

  //   public static final double STEER_ENC_POS_TO_METERS =
  //       (2 * Math.PI); // factor of steer encoder to meters(conversion factor)
  //   public static final double STEER_ENC_VEL_TO_METERS =
  //       (2 * Math.PI) / 60; // factor of vel to meters

  //   public static final boolean STEER_ENC_INVERTED = true;
  //   public static final boolean DRIVE_INVERTED = true;
  //   public static final boolean STEER_INVERTED = true;

  //   public static final double K_DRIVE_P = 0.1; // TODO
  //   public static final double K_DRIVE_I = 0.0; // TODO
  //   public static final double K_DRIVE_D = 0; // TODO
  //   public static final double K_DRIVE_FF = 1 / DRIVE_WHEEL_FREESPEED;

  //   public static final double K_STEER_P = 0.7; // TODO
  //   public static final double K_STEER_I = 0.0; // TODO
  //   public static final double K_STEER_D = 0.0; // TODO
  //   public static final double K_STEER_FF = 0.0; // TODO

  //   public static final int DRIVE_CURRENT_LIMIT = 50; // amps
  //   public static final int STEER_CURRENT_LIMIT = 20; // amps

  //   public static final IdleMode DRIVE_IDLE = IdleMode.kBrake;
  //   public static final IdleMode STEER_IDLE = IdleMode.kCoast;

  //   public static final double STEER_ENC_PID_MIN = 0.0;
  //   public static final double STEER_ENC_PID_MAX = STEER_ENC_POS_TO_METERS; // TODO

  //   public static final double STEER_OUT_MIN = -1;
  //   public static final double STEER_OUT_MAX = 1;
  //   public static final double DRIVE_OUT_MIN = -1;
  //   public static final double DRIVE_OUT_MAX = 1;

  // #endregion Test Chassis

  // #region Comp Chassis

  public static final double WHEEL_WIDTH =
      Units.inchesToMeters(16.6); // Make sure this is from the wheel's center of rotation
  public static final double WHEEL_DIAMETER =
      Units.inchesToMeters(4); // TODO measure more precisely

  // public static final  WHEEL_POSITIONS =  TODO

  public static final ModuleConfig[] MOD_CONFIGS = {
    new ModuleConfig(1, 2, 9, 0), // LU
    new ModuleConfig(3, 4, 10, 0), // RU
    new ModuleConfig(5, 6, 11, 0), // LD
    new ModuleConfig(7, 8, 12, 0) // RD
  };

  public static final int PIGEON_ID = 0;

  public static final double MAX_SPEED = 5.7912; // TODO
  public static final double MAX_ANGULAR_VELOCITY = 8; // TODO set temporarily, to look into later

  // WHEELS

  public static final double DRIVE_GEAR_RATIO = (2.8125);

  public static final double DRIVE_MOTOR_FREESPEED_RPS = 5676 / 60; // Free RPM of NEO to RPS
  public static final double DRIVE_WHEEL_FREESPEED = 
      (DRIVE_MOTOR_FREESPEED_RPS * (WHEEL_DIAMETER * Math.PI))
          / DRIVE_GEAR_RATIO; // Converted for wheel

  public static final double DRIVE_ENC_TO_METERS = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
  public static final double DRIVE_ENC_VEL_TO_METERS =
      ((WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO) / 60;

  public static final double STEER_ENC_POS_TO_METERS =
      (2 * Math.PI); // factor of steer encoder to meters(conversion factor)
  public static final double STEER_ENC_VEL_TO_METERS =
      (2 * Math.PI) / 60; // factor of vel to meters

  public static final boolean STEER_ENC_INVERTED = true;
  public static final boolean DRIVE_INVERTED = true;
  public static final boolean STEER_INVERTED = true;

  public static final double K_DRIVE_P = 0.1; // TODO
  public static final double K_DRIVE_I = 0.0; // TODO
  public static final double K_DRIVE_D = 0; // TODO
  public static final double K_DRIVE_FF = 1 / DRIVE_WHEEL_FREESPEED;

  public static final double K_STEER_P = 0.7; // TODO
  public static final double K_STEER_I = 0.0; // TODO
  public static final double K_STEER_D = 0.0; // TODO
  public static final double K_STEER_FF = 0.0; // TODO

  public static final int DRIVE_CURRENT_LIMIT = 50; // amps
  public static final int STEER_CURRENT_LIMIT = 20; // amps

  public static final IdleMode DRIVE_IDLE = IdleMode.kBrake;
  public static final IdleMode STEER_IDLE = IdleMode.kCoast;

  public static final double STEER_ENC_PID_MIN = 0.0;
  public static final double STEER_ENC_PID_MAX = STEER_ENC_POS_TO_METERS; // TODO

  public static final double STEER_OUT_MIN = -1;
  public static final double STEER_OUT_MAX = 1;
  public static final double DRIVE_OUT_MIN = -1;
  public static final double DRIVE_OUT_MAX = 1;

  // #endregion Comp Chassis
}
