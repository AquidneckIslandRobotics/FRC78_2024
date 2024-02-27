// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Structs.Range;
import frc.robot.classes.Util;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VarShootPrime extends Command {
  private Wrist wrist;
  private Shooter shooter;
  private Elevator elevator;
  private PoseEstimator poseEstimator;
  private Supplier<Translation2d> speakerTranslation;

  // Translation of where the note exits in the XZ plane (side view)
  private final Translation2d shooterXZTrans;

  private final Range velRange; // Range of velocity from min distance to max distance
  private final Range distRange; // Range of distance from min distance to max distance
  private final double heightLengthCoeff;
  private final double RPM_MPS;

  /** Creates a new VarShootPrime. */
  public VarShootPrime(
      Wrist wrist,
      Shooter shooter,
      Elevator elevator,
      PoseEstimator poseEstimator,
      Translation2d shooterXZTrans,
      Range velRange,
      Range distRange,
      double thetaCoeff,
      double RPM_MPS) {
    this.wrist = wrist;
    this.shooter = shooter;
    this.elevator = elevator;
    this.poseEstimator = poseEstimator;
    this.speakerTranslation = Constants.SPEAKER_TRANSLATION;
    this.shooterXZTrans = shooterXZTrans;
    this.velRange = velRange;
    this.distRange = distRange;
    this.heightLengthCoeff = thetaCoeff;
    this.RPM_MPS = RPM_MPS;

    addRequirements(wrist, shooter);
  }

  @Override
  public void execute() {
    Pose2d pose = poseEstimator.getFusedPose();

    // Distance and height to speaker
    double l = pose.getTranslation().getDistance(speakerTranslation.get()) - shooterXZTrans.getX();
    double h =
        (Constants.SPEAKER_HEIGHT - shooterXZTrans.getY())
            - Units.inchesToMeters(elevator.getElevatorPos());
    // Calculate velocity based on lerping within the velocity range based on the distance range
    // double v = Util.lerp(Util.clamp(h, distRange) / distRange.getRange(), velRange);
    double v = calcVel();
    double theta = calcTheta(Constants.GRAVITY, l, h, v);
    theta = Units.radiansToDegrees(theta);
    double modify = Util.lerp(l, distRange) * heightLengthCoeff;
    theta += modify;
    wrist.setToTarget(theta);
    Logger.recordOutput("VarShootPrime theta", theta);
    Logger.recordOutput("VarShootPrime modify", modify);
    Logger.recordOutput("VarShootPrime h", h);
    Logger.recordOutput("VarShootPrime v", v);
    Logger.recordOutput("VarShootPrime l", l);
    Logger.recordOutput(
        "REAL VALUES PROBABLY", Units.radiansToDegrees(calcTheta(9.81, 2, 1.4, 18)));

    shooter.setSpeed(v / RPM_MPS);
  }

  public double calcVel() {
    return velRange.max;
  }

  // Source? It was revealed to me by a wise tree in a dream
  // JK this https://en.wikipedia.org/wiki/Projectile_motion
  private double calcTheta(double g, double l, double h, double v) {
    double sqrt = Math.pow(v, 4) - (g * ((g * l * l) + (2 * h * v * v)));
    Logger.recordOutput("sqrt", sqrt);
    double nominator = (v * v) - Math.sqrt(sqrt);
    double denominator = g * l;
    Logger.recordOutput("denominator", denominator);

    return Math.atan(nominator / denominator);
  }
}
