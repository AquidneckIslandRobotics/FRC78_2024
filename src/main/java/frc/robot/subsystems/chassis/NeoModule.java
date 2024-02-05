// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.classes.ModuleConfig;

/** Neo implementation of SwerveModule */
public class NeoModule implements SwerveModule {

  protected ModuleConfig config;

  protected CANSparkMax drive;
  protected CANSparkMax steer;

  protected SparkPIDController drivePID;
  protected SparkPIDController steerPID;
  private RelativeEncoder driveEnc;
  private AbsoluteEncoder steerEnc;

  private SimpleMotorFeedforward driveFeedforward;

  public NeoModule(ModuleConfig config) {
    this.config = config;

    drive = new CANSparkMax(this.config.driveID, MotorType.kBrushless);
    steer = new CANSparkMax(this.config.steerID, MotorType.kBrushless);

    driveEnc = drive.getEncoder();
    steerEnc = steer.getAbsoluteEncoder(Type.kDutyCycle);
    drivePID = drive.getPIDController();
    steerPID = steer.getPIDController();

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
    this.driveFeedforward =
        new SimpleMotorFeedforward(
            config.driveClosedLoopParameters.kS,
            config.driveClosedLoopParameters.kV,
            config.driveClosedLoopParameters.kA);

    // Set the PID gains for the turning motor
    steerPID.setP(config.steerClosedLoopParameters.kP);
    steerPID.setI(config.steerClosedLoopParameters.kI);
    steerPID.setD(config.steerClosedLoopParameters.kD);

    drive.setSmartCurrentLimit(config.driveCurrentLimit);
    steer.setSmartCurrentLimit(config.steerCurrentLimit);

    drive.enableVoltageCompensation(config.nominalVoltage);
    steer.enableVoltageCompensation(config.nominalVoltage);

    drive.setIdleMode(config.driveIdleMode);
    steer.setIdleMode(config.steerIdleMode);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    drive.burnFlash();
    steer.burnFlash();

    driveEnc.setPosition(0);
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

  /**
   * Sets the desired state of the module. Should be used if you want to set both the velocity and
   * rotation at the same time
   *
   * @param state The desired state of the module
   */
  @Override
  public void setState(SwerveModuleState state) {
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getSteerPosition());

    // Sets the PID goals to the desired states
    double driveFf =
        driveFeedforward.calculate(
            driveEnc.getVelocity(), optimizedState.speedMetersPerSecond, 0.02);
    drivePID.setReference(
        optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, driveFf);
    steerPID.setReference(optimizedState.angle.getRotations(), CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber(config.driveID + " setting rot", optimizedState.angle.getRotations());
    SmartDashboard.putNumber(config.driveID + " getting rot", steerEnc.getPosition());
    SmartDashboard.putNumber(config.driveID + "getting speed", getDriveVelocity());
    SmartDashboard.putNumber(config.driveID + "setting speed", optimizedState.speedMetersPerSecond);
  }

  public void openLoopDiffDrive(double voltage) {
    steerPID.setReference(0, ControlType.kPosition);
    drive.setVoltage(voltage);
  }

  /*
   * Mutate these each time we log so that we aren't creating objects constantly
   */
  private final MutableMeasure<Voltage> mutableAppliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Distance> mutableDistance = MutableMeasure.mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> mutableVelocity =
      MutableMeasure.mutable(MetersPerSecond.of(0));

  public void logMotor(SysIdRoutineLog log) {
    log.motor("motor#" + config.driveID)
        // Log voltage
        .voltage(
            /* getAppliedOutput returns the duty cycle which is from [-1, +1].
            We multiply this by the voltage going into the spark max,
            called the bus voltage to receive the output voltage */
            mutableAppliedVoltage.mut_replace(
                drive.getAppliedOutput() * drive.getBusVoltage(), Volts))
        // the drive encoder has the necessary position and velocity conversion factors already set
        .linearVelocity(mutableVelocity.mut_replace(driveEnc.getVelocity(), MetersPerSecond))
        .linearPosition(mutableDistance.mut_replace(driveEnc.getPosition(), Meters));
  }
}
