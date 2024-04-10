// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs.Range;
import frc.robot.classes.Util;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoAimShot extends Command {
  private Shooter shooter;
  private Wrist wrist;
  private Elevator elevator;
  private PoseEstimator poseEstimator;
  private Translation2d speakerTranslation;

  // Translation of where the note exits in the XZ plane (side view)
  private final Translation2d shooterXZTrans;

  private final DoubleSupplier shooterVel; // Range of velocity from min distance to max distance
  private final Range distRange; // Range of distance from min distance to max distance
  private final double heightLengthCoeff;
  private final double RPM_MPS;
  private final double defaultAngle;

  /** Creates a new VarShootPrime. */
  public AutoAimShot(
      Shooter shooter,
      Wrist wrist,
      Elevator elevator,
      PoseEstimator poseEstimator,
      Translation2d shooterXZTrans,
      DoubleSupplier shooterVel,
      Range distRange,
      double thetaCoeff,
      double RPM_MPS,
      double defaultAngle) {
    this.shooter = shooter;
    this.wrist = wrist;
    this.elevator = elevator;
    this.poseEstimator = poseEstimator;
    this.shooterXZTrans = shooterXZTrans;
    this.shooterVel = shooterVel;
    this.distRange = distRange;
    this.heightLengthCoeff = thetaCoeff;
    this.RPM_MPS = RPM_MPS;
    this.defaultAngle = defaultAngle;

    addRequirements(wrist, shooter);
  }

  @Override
  public void initialize() {
    speakerTranslation =
        DriverStation.getAlliance().isPresent()
            ? (DriverStation.getAlliance().get() == Alliance.Red
                ? Constants.RED_SPEAKER_POSE
                : Constants.BLUE_SPEAKER_POSE)
            : Constants.BLUE_SPEAKER_POSE;
  }

  @Override
  public void execute() {
    Pose2d pose = poseEstimator.getFusedPose();

    // Distance and height to speaker
    double distanceFromSpeaker =
        pose.getTranslation().getDistance(speakerTranslation) - shooterXZTrans.getX();

    if (distanceFromSpeaker < 2) {
      shooter.setSpeed(3000);
    } else {
      shooter.setSpeed(4000);
    }
    double heightToShooter =
        (Constants.SPEAKER_HEIGHT.in(Meters) - shooterXZTrans.getY())
            - Units.inchesToMeters(elevator.getElevatorPos());
    // Calculate velocity based on lerping within the velocity range based on the distance range
    // double currentShooterSpeed = Util.lerp(Util.clamp(heightToShooter, distRange) /
    // distRange.getRange(), velRange);
    double currentShooterSpeed = shooterVel.getAsDouble() * RPM_MPS;
    double theta =
        calcTheta(Constants.GRAVITY, distanceFromSpeaker, heightToShooter, currentShooterSpeed);
    theta = Units.radiansToDegrees(theta);
    double modify = Util.lerp(distanceFromSpeaker, distRange) * heightLengthCoeff;
    theta += modify;
    Logger.recordOutput("VarShootPrime theta", theta);
    Logger.recordOutput("VarShootPrime modify", modify);
    Logger.recordOutput("VarShootPrime heightToShooter", heightToShooter);
    Logger.recordOutput("VarShootPrime currentShooterSpeed", currentShooterSpeed);
    Logger.recordOutput("VarShootPrime l", distanceFromSpeaker);

    wrist.setToTarget(theta);
  }

  // Source? It was revealed to me by a wise tree in a dream
  // JK this https://en.wikipedia.org/wiki/Projectile_motion
  private double calcTheta(double g, double l, double h, double v) {
    double sqrt = v * v * v * v - (g * ((g * l * l) + (2 * h * v * v)));
    double numerator = (v * v) - Math.sqrt(sqrt);
    double denominator = g * l;

    return Math.atan(numerator / denominator);
  }
}
