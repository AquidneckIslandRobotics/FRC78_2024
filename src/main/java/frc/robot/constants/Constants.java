// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

/** This is the constants file for all of the common constants */
public final class Constants {
  public static final double JOYSTICK_DEADBAND = 0.15;
  public static final double TRIGGER_DEADBAND = 0.1;

  public static final double UP_ADJUST = 0.5;
  public static final double DOWN_ADJUST = 0.25;

  public static final Translation2d BLUE_ORBIT_POSE = new Translation2d(0, 5.5);
  public static final Translation2d RED_ORBIT_POSE = new Translation2d(16.5, 5.5);
  // TODO should revise and precisely measure these values, they are estimates
  public static final Translation2d BLUE_SPEAKER_POSE = new Translation2d(0, 5.5);
  public static final Translation2d RED_SPEAKER_POSE = new Translation2d(16.5, 5.5);
  public static final double SPEAKER_HEIGHT = 2.1;
  public static final Supplier<Translation2d> SPEAKER_TRANSLATION =
      () ->
          DriverStation.getAlliance().isPresent()
              ? (DriverStation.getAlliance().get() == Alliance.Red
                  ? RED_SPEAKER_POSE
                  : BLUE_SPEAKER_POSE)
              : BLUE_SPEAKER_POSE;

  public static final Transform2d BLUE_AMP =
      new Transform2d(1.83, 7.47, Rotation2d.fromDegrees(90));
  public static final Transform2d RED_AMP =
      new Transform2d(14.65, 7.47, Rotation2d.fromDegrees(90));
  public static final Supplier<Transform2d> AMP_TRANSFORM =
      () ->
          DriverStation.getAlliance().isPresent()
              ? (DriverStation.getAlliance().get() == Alliance.Red ? RED_AMP : BLUE_AMP)
              : BLUE_AMP;

  public static final double ORBIT_RADIUS = 2;
  public static final double ORBIT_RADIUS_MARGIN = 1.0;

  public static final double GRAVITY = 9.807;
}
