// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.classes.ModuleConfig;
import frc.robot.classes.Structs.FFConstants;
import frc.robot.classes.Util;
import org.littletonrobotics.junction.Logger;

/** Neo implementation of SwerveModule */
public class NeoModule implements SwerveModule {

  protected ModuleConfig config;

  protected int driveID;
  protected int steerID;
  protected CANSparkMax drive;
  protected CANSparkMax steer;

  protected SparkPIDController drivePID;
  protected SparkPIDController steerPID;
  protected SimpleMotorFeedforward driveFF;
  private RelativeEncoder driveEnc;
  private AbsoluteEncoder steerEnc;

  private SwerveModuleState settingState;
  private SwerveModuleState realState;
  private SwerveModuleState optimizedState;

  public NeoModule(int driveID, int steerID, ModuleConfig config, FFConstants ffConstants) {
    this.config = config;

    this.driveID = driveID;
    this.steerID = steerID;
    drive = new CANSparkMax(this.driveID, MotorType.kBrushless);
    steer = new CANSparkMax(this.steerID, MotorType.kBrushless);

    driveEnc = drive.getEncoder();
    steerEnc = steer.getAbsoluteEncoder(Type.kDutyCycle);
    drivePID = drive.getPIDController();
    steerPID = steer.getPIDController();
    driveFF = new SimpleMotorFeedforward(ffConstants.kS, ffConstants.kV, ffConstants.kA);

    settingState = new SwerveModuleState();
    realState = new SwerveModuleState();
    optimizedState = new SwerveModuleState();

    initialize();
  }

  /** Initializes the NEO module. This should be called in the constructor of the module class. */
  @Override
  public void initialize() {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out
    drive.restoreFactoryDefaults();
    steer.restoreFactoryDefaults();
    drivePID.setFeedbackDevice(driveEnc);
    steerPID.setFeedbackDevice(steerEnc);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEnc.setPositionConversionFactor(config.drivePositionConversionFactor);
    driveEnc.setVelocityConversionFactor(config.driveVelocityConversionFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    steerEnc.setPositionConversionFactor(config.steerPositionConversionFactor);
    steerEnc.setVelocityConversionFactor(config.steerVelocityConversionFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    steerEnc.setInverted(config.steerEncoderInverted);
    steer.setInverted(config.steerMotorInverted);
    drive.setInverted(config.driveMotorInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    steerPID.setPositionPIDWrappingEnabled(true);
    steerPID.setPositionPIDWrappingMinInput(config.steerEncoderPidMin);
    steerPID.setPositionPIDWrappingMaxInput(config.steerEncoderPidMax);

    // Set the PID gains for the driving motor
    drivePID.setP(config.driveClosedLoopParameters.kP);
    drivePID.setI(config.driveClosedLoopParameters.kI);
    drivePID.setD(config.driveClosedLoopParameters.kD);
    drivePID.setFF(config.driveClosedLoopParameters.kF);

    // Set the PID gains for the turning motor
    steerPID.setP(config.steerClosedLoopParameters.kP);
    steerPID.setI(config.steerClosedLoopParameters.kI);
    steerPID.setD(config.steerClosedLoopParameters.kD);
    steerPID.setFF(config.steerClosedLoopParameters.kF);

    drive.setSmartCurrentLimit(config.driveCurrentLimit);
    steer.setSmartCurrentLimit(config.steerCurrentLimit);

    drive.enableVoltageCompensation(config.nominalVoltage);
    steer.enableVoltageCompensation(config.nominalVoltage);

    drive.setIdleMode(config.driveIdleMode);
    steer.setIdleMode(config.steerIdleMode);

    Util.setRevStatusRates(steer, 10, 20, 32767, 32767, 32767, 100, 32767, 32767);
    Util.setRevStatusRates(drive, 10, 20, 20, 32767, 32767, 32767, 32767, 32767);

    settingState.angle = Rotation2d.fromRotations(steerEnc.getPosition());
    driveEnc.setPosition(0);
  }

  @Override
  public void setBrake(Boolean y) {
    if (y) {
      drive.setIdleMode(IdleMode.kBrake);
      steer.setIdleMode(IdleMode.kBrake);
    } else {
      drive.setIdleMode(IdleMode.kCoast);
      steer.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * @return The velocity of the drive encoder, in meters per second
   */
  @Override
  public double getDriveVelocity() {
    return driveEnc.getVelocity();
  }

  /**
   * @return The relative position of the drive encoder, in meters
   */
  @Override
  public double getDrivePosition() {
    return driveEnc.getPosition();
  }

  /**
   * @return The absolute position of the drive encoder, as a {@link Rotation2d#Rotation2d
   *     Rotation2d}
   */
  @Override
  public Rotation2d getSteerPosition() {
    return Rotation2d.fromRotations(steerEnc.getPosition());
  }

  /**
   * @return The state of the module as a {@link SwerveModuleState#SwerveModuleState
   *     SwerveModuleState}
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerPosition());
  }

  /**
   * @return The position of the module as a {@link SwerveModulePosition#SwerveModulePosition
   *     SwerveModulePosition}
   */
  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getSteerPosition());
  }

  @Override
  public SwerveModuleState getOptimizedState() {
    return optimizedState;
  }

  @Override
  public SwerveModuleState getRealState() {
    return realState;
  }

  /**
   * Sets the desired state of the module. Should be used if you want to set both the velocity and
   * rotation at the same time
   *
   * @param state The desired state of the module
   */
  @Override
  public void setState(SwerveModuleState state) {
    // Optimize the reference state to avoid spinning further than 90 degrees.
    optimizedState = SwerveModuleState.optimize(state, getSteerPosition());
    double speedModifier =
        Math.abs(Math.cos(((optimizedState.angle.getRadians()) - getSteerPosition().getRadians())));
    optimizedState.speedMetersPerSecond *= speedModifier;
    Logger.recordOutput("Swerve speed modifier", speedModifier);

    // Sets the PID goals to the desired states
    drivePID.setReference(
        optimizedState.speedMetersPerSecond,
        CANSparkMax.ControlType.kVelocity,
        0,
        driveFF.calculate(optimizedState.speedMetersPerSecond),
        ArbFFUnits.kVoltage);
    steerPID.setReference(optimizedState.angle.getRotations(), CANSparkMax.ControlType.kPosition);

    settingState = state;
    realState.speedMetersPerSecond = getDriveVelocity();
    realState.angle = getSteerPosition();
    // (-pi to pi) to a zero to 1
    // Logger.recordOutput(driveID + " drive meters", driveEnc.getPosition());
    Logger.recordOutput(
        driveID + "steer err",
        optimizedState.angle.getRotations() - realState.angle.getRotations());
  }

  public void openLoopDiffDrive(double voltage) {
    steerPID.setReference(0, ControlType.kPosition);
    drive.setVoltage(voltage);
  }

  @Override
  public void enableBrakeMode() {
    drive.setIdleMode(CANSparkBase.IdleMode.kBrake);
    steer.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }

  @Override
  public void enableCoastMode() {
    drive.setIdleMode(CANSparkBase.IdleMode.kCoast);
    steer.setIdleMode(CANSparkBase.IdleMode.kCoast);
  }
}
