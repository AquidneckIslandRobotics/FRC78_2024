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
import frc.robot.classes.ProjectileMotionEquations;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.chassis.PoseEstimator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class VarFeedPrime extends Command {
  private Shooter shooter;
  private Elevator elevator;
  private PoseEstimator poseEstimator;
  private Translation2d plopTranslation;

  // Translation of where the note exits in the XZ plane (side view)
  private final Translation2d shooterXZTrans;

  private final DoubleSupplier wristAngle; // Range of velocity from min distance to max distance
  private final double RPM_MPS;

  /** Creates a new VarShootPrime. */
  public VarFeedPrime(
      Shooter shooter,
      Elevator elevator,
      PoseEstimator poseEstimator,
      Translation2d shooterXZTrans,
      DoubleSupplier wristAngle,
      double RPM_MPS) {
    this.shooter = shooter;
    this.elevator = elevator;
    this.poseEstimator = poseEstimator;
    this.shooterXZTrans = shooterXZTrans;
    this.wristAngle = wristAngle;
    this.RPM_MPS = RPM_MPS;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    plopTranslation =
        DriverStation.getAlliance().isPresent()
            ? (DriverStation.getAlliance().get() == Alliance.Red
                ? Constants.RED_PLOP_POSE
                : Constants.BLUE_PLOP_POSE)
            : Constants.BLUE_PLOP_POSE;
  }

  @Override
  public void execute() {
    Pose2d pose = poseEstimator.getFusedPose();

    // Distance and height to speaker
    double l = pose.getTranslation().getDistance(plopTranslation) - shooterXZTrans.getX();
    double h = shooterXZTrans.getY() - Units.inchesToMeters(elevator.getElevatorPos());

    double theta = Math.toRadians(wristAngle.getAsDouble());
    double calcV =
        ProjectileMotionEquations.calculateLaunchVelocity(Constants.GRAVITY, l, -h, theta);
    double v = Double.isNaN(calcV) ? 0 : calcV;

    Logger.recordOutput("VarFeedPrime theta", theta);
    Logger.recordOutput("VarFeedPrime h", h);
    Logger.recordOutput("VarFeedPrime v", v);
    Logger.recordOutput("VarFeedPrime l", l);

    shooter.setSpeed(v / RPM_MPS);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
  }
}
