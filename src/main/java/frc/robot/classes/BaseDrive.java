// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.DoubleSupplier;

/**
 * Base driving class, takes in joysticks and performs all processing, outputting chassis speeds
 * that can then be fed into further processing
 */
public class BaseDrive {
  public XboxController controller;
  public Structs.MotionLimits motionLimits;
  public Structs.RateLimits rateLimit;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;
  private final DoubleSupplier lTriggerSupplier;
  private final DoubleSupplier rTriggerSupplier;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;

  private final ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  public BaseDrive(
      XboxController controller, Structs.MotionLimits motionLimits, Structs.RateLimits rateLimits) {
    this.controller = controller;
    this.motionLimits = motionLimits;
    this.rateLimit = rateLimits;
    this.xSupplier = () -> -controller.getLeftY();
    this.ySupplier = () -> -controller.getLeftX();
    this.rotSupplier = () -> -controller.getRightX();
    this.lTriggerSupplier = controller::getLeftTriggerAxis;
    this.rTriggerSupplier = controller::getRightTriggerAxis;

    xLimiter = new SlewRateLimiter(rateLimits.translationRateLimit);
    yLimiter = new SlewRateLimiter(rateLimits.translationRateLimit);
    thetaLimiter = new SlewRateLimiter(rateLimits.rotationRateLimit);
  }

  public ChassisSpeeds calculateChassisSpeeds() {
    double trigger =
        Util.triggerAdjust(
            Util.modifyTrigger(lTriggerSupplier.getAsDouble()),
            Util.modifyTrigger(rTriggerSupplier.getAsDouble()));

    double speed = trigger * motionLimits.maxSpeed;
    double angularSpeed = trigger * motionLimits.maxAngularSpeed;

    double x = xLimiter.calculate(Util.modifyJoystick(xSupplier.getAsDouble()) * speed);
    double y = yLimiter.calculate(Util.modifyJoystick(ySupplier.getAsDouble()) * speed);
    double rot =
        thetaLimiter.calculate(Util.modifyJoystick(rotSupplier.getAsDouble()) * angularSpeed);

    chassisSpeeds.vxMetersPerSecond = x;
    chassisSpeeds.vyMetersPerSecond = y;
    chassisSpeeds.omegaRadiansPerSecond = rot;
    return chassisSpeeds;
  }
}
