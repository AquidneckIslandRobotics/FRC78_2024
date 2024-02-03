// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

/** Utility class */
public class Util {
  /**
   * Adjusts the speeds of the given input depending on trigger input, with left trigger decreasing
   * speed and RT increasing.
   *
   * <p>Default speed = 1 - up adjust Full left trigger = (1 - upAdjust) - down adjust Full right
   * trigger = 1
   *
   * @param in
   * @return Adjusted speed
   */
  public static double triggerAdjust(double down, double up) {
    double triggers =
        (1 - Constants.UP_ADJUST) + (up * Constants.UP_ADJUST) - (down * Constants.DOWN_ADJUST);
    return triggers;
  }

  public static double modifyTrigger(double value) {
    return MathUtil.applyDeadband(value, Constants.TRIGGER_DEADBAND);
  }

  /**
   * Processes the given joystick axis value, applying deadband and squaring it
   *
   * @param value
   * @return
   */
  public static double modifyJoystick(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, Constants.JOYSTICK_DEADBAND);
    // Square the axis
    // value = Math.copySign(value * value, value);
    return value;
  }

  public static Structs.Vector2 normalizeVector(Structs.Vector2 vector) {
    double magnitude = Math.sqrt(vector.x * vector.x + vector.y * vector.y);
    return new Structs.Vector2(vector.x / magnitude, vector.y / magnitude);
  }
}
