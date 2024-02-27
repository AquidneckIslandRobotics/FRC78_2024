// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Add your docs here. */
public class CalcAimAngle {
  public static double calcAimAngle(
      Supplier<Translation2d> speakerTranslation,
      Supplier<Pose2d> robotPose,
      Supplier<Transform2d> robotVel,
      DoubleSupplier v,
      DoubleSupplier theta,
      double velDistCoeff) {

    double distGoal = robotPose.get().getTranslation().getDistance(speakerTranslation.get());
    double tAtGoal =
        distGoal / (v.getAsDouble() * Math.cos(Units.degreesToRadians(theta.getAsDouble())));
    double lateralDisplacement =
        -robotVel.get().getTranslation().getY() * tAtGoal; // Negate because +Y is left hand

    return 0;
  }
}
