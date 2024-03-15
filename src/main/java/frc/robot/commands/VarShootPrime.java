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
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class VarShootPrime extends Command {
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

  private double theta;
  private double v;
  private final double defaultAngle;

  /** Creates a new VarShootPrime. */
  public VarShootPrime(
      Wrist wrist,
      Elevator elevator,
      PoseEstimator poseEstimator,
      Translation2d shooterXZTrans,
      DoubleSupplier shooterVel,
      Range distRange,
      double thetaCoeff,
      double RPM_MPS,
      double defaultAngle) {
    this.wrist = wrist;
    this.elevator = elevator;
    this.poseEstimator = poseEstimator;
    this.shooterXZTrans = shooterXZTrans;
    this.shooterVel = shooterVel;
    this.distRange = distRange;
    this.heightLengthCoeff = thetaCoeff;
    this.RPM_MPS = RPM_MPS;
    this.defaultAngle = defaultAngle;

    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    theta = 0;
    v = 0;
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
    double l = pose.getTranslation().getDistance(speakerTranslation) - shooterXZTrans.getX();
    double h =
        (Constants.SPEAKER_HEIGHT.in(Meters) - shooterXZTrans.getY())
            - Units.inchesToMeters(elevator.getElevatorPos());
    // Calculate velocity based on lerping within the velocity range based on the distance range
    // double v = Util.lerp(Util.clamp(h, distRange) / distRange.getRange(), velRange);
    double v = shooterVel.getAsDouble() * RPM_MPS;
    v += poseEstimator.getEstimatedVel().getX();
    double theta = calcTheta(Constants.GRAVITY, l, h, v);
    if (theta == Double.NaN) theta = defaultAngle;
    theta = Units.radiansToDegrees(theta);
    double modify = Util.lerp(l, distRange) * heightLengthCoeff;
    theta += modify;
    Logger.recordOutput("VarShootPrime theta", theta);
    Logger.recordOutput("VarShootPrime modify", modify);
    Logger.recordOutput("VarShootPrime h", h);
    Logger.recordOutput("VarShootPrime v", v);
    Logger.recordOutput("VarShootPrime l", l);

    wrist.setToTarget(theta);
  }

  public double getV() {
    return v;
  }

  public double getTheta() {
    return theta;
  }

  // Source? It was revealed to me by a wise tree in a dream
  // JK this https://en.wikipedia.org/wiki/Projectile_motion
  private double calcTheta(double g, double l, double h, double v) {
    double sqrt = Math.pow(v, 4) - (g * ((g * l * l) + (2 * h * v * v)));
    double numerator = (v * v) - Math.sqrt(sqrt);
    double denominator = g * l;

    return Math.atan(numerator / denominator);
  }
}
