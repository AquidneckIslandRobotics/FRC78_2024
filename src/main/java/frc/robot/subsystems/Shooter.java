// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.classes.Structs.FFConstants;
import frc.robot.classes.Structs.Range;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private TalonFX shooterTOP;
  private TalonFX shooterBOTTOM;

  private final VelocityVoltage shooterTopVV;
  private final VelocityVoltage shooterBottomVV;

  private final ShooterConfig config;

  private final NetworkTableEntry slowShot;

  private final double slowShotSpeed = 500;

  private final VoltageOut sysIdVoltage = new VoltageOut(0, false, false, false, false);
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1).per(Second),
              Volts.of(7),
              Seconds.of(10),
              (state) -> SignalLogger.writeString("sysid-test-state-shooter", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltageMeasure) -> {
                sysIdVoltage.withOutput(voltageMeasure.in(Volts));
                this.shooterTOP.setControl(sysIdVoltage);
                this.shooterBOTTOM.setControl(sysIdVoltage);
              },
              null,
              this,
              "shooter"));

  /** Creates a new Shooter. */
  public Shooter(ShooterConfig config) {
    this.config = config;
    shooterTOP = new TalonFX(config.flywheelTopID);
    shooterBOTTOM = new TalonFX(config.flywheelBottomID);

    shooterTOP.getConfigurator().apply(new TalonFXConfiguration());
    shooterBOTTOM.getConfigurator().apply(new TalonFXConfiguration());

    var shooterTopConfig = new Slot0Configs();
    shooterTOP.setInverted(config.flywheelTopInverted);
    shooterTopConfig.kS = config.flywheelTopFF.kS;
    shooterTopConfig.kV = config.flywheelTopFF.kV;
    shooterTopConfig.kP = config.flywheelTopPID.kP;
    shooterTopConfig.kI = config.flywheelTopPID.kI;
    shooterTopConfig.kD = config.flywheelTopPID.kD;

    shooterTOP.getConfigurator().apply(shooterTopConfig);

    var shooterBottomConfig = new Slot0Configs();
    shooterBOTTOM.setInverted(config.flywheelBottomInverted);
    shooterBottomConfig.kS = config.flywheelBottomFF.kS;
    shooterBottomConfig.kV = config.flywheelBottomFF.kV;
    shooterBottomConfig.kP = config.flywheelBottomPID.kP;
    shooterBottomConfig.kI = config.flywheelBottomPID.kI;
    shooterBottomConfig.kD = config.flywheelBottomPID.kD;

    shooterBOTTOM.getConfigurator().apply(shooterBottomConfig);

    shooterTopVV = new VelocityVoltage(0).withSlot(0);
    shooterBottomVV = new VelocityVoltage(0).withSlot(0);

    shooterTOP.getVelocity().setUpdateFrequency(50);
    shooterBOTTOM.getVelocity().setUpdateFrequency(50);

    shooterTOP.optimizeBusUtilization();
    shooterBOTTOM.optimizeBusUtilization();

    slowShot = SmartDashboard.getEntry("shooter/slowShot");
    slowShot.setPersistent();
    slowShot.setDefaultBoolean(false);
  }

  private void configureMotorsBeforeSysId() {
    shooterTOP.getVelocity().setUpdateFrequency(1000);
    shooterTOP.getMotorVoltage().setUpdateFrequency(1000);
    shooterTOP.getPosition().setUpdateFrequency(1000);
    shooterBOTTOM.getVelocity().setUpdateFrequency(1000);
    shooterBOTTOM.getMotorVoltage().setUpdateFrequency(1000);
    shooterBOTTOM.getPosition().setUpdateFrequency(1000);
  }

  public void configureMotorsAfterSysId() {
    shooterTOP.getVelocity().setUpdateFrequency(50);
    shooterTOP.getMotorVoltage().setUpdateFrequency(0);
    shooterTOP.getPosition().setUpdateFrequency(0);
    shooterBOTTOM.getVelocity().setUpdateFrequency(50);
    shooterBOTTOM.getMotorVoltage().setUpdateFrequency(0);
    shooterBOTTOM.getPosition().setUpdateFrequency(0);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureMotorsBeforeSysId)
        .andThen(sysIdRoutine.quasistatic(direction))
        .andThen(runOnce(this::configureMotorsAfterSysId));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureMotorsBeforeSysId)
        .andThen(sysIdRoutine.dynamic(direction))
        .andThen(runOnce(this::configureMotorsAfterSysId));
  }

  public boolean isAtSpeed(double threshold) {
    return ((getVelocity() >= shooterTopVV.Velocity * threshold) && (shooterTopVV.Velocity != 0));
  }

  private void setPIDReferenceTOP(double setPoint) {
    shooterTOP.setControl(
        shooterTopVV.withVelocity(setPoint / 60).withFeedForward(config.flywheelTopFF.kFF));
  }

  private void setPIDReferenceBOTTOM(double setPoint) {
    shooterBOTTOM.setControl(
        shooterBottomVV.withVelocity(setPoint / 60).withFeedForward(config.flywheelTopFF.kFF));
  }

  private void setPIDReferenceBOTH(double setPoint) {
    setPIDReferenceTOP(setPoint);
    setPIDReferenceBOTTOM(setPoint);
  }

  public double getVelocity() {
    return (shooterTOP.getVelocity().getValueAsDouble() * 0.5)
        + (shooterBOTTOM.getVelocity().getValueAsDouble() * 0.5);
  }

  public Command setSpeed(double setPoint) {
    return this.runOnce(
        () -> {
          if (slowShot.getBoolean(false)) {
            this.setPIDReferenceBOTH(Math.min(setPoint, slowShotSpeed));
          } else {
            this.setPIDReferenceBOTH(setPoint);
          }
        });
  }

  @Override
  public void periodic() {
    // TODO is this reading velocity encoder or the setting velocity?
    Logger.recordOutput("Shooter Top", shooterTOP.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter Bottom", shooterBOTTOM.getVelocity().getValueAsDouble());
  }

  public static class ShooterConfig {
    public final int flywheelTopID;
    public final int flywheelBottomID;
    public final Boolean flywheelTopInverted;
    public final Boolean flywheelBottomInverted;
    public final Range flywheelTopMinMax;
    public final Range flywheelBottomMinMax;
    public final PIDConstants flywheelTopPID;
    public final PIDConstants flywheelBottomPID;
    public final FFConstants flywheelTopFF;
    public final FFConstants flywheelBottomFF;

    public ShooterConfig(
        int flywheelTopID,
        int flywheelBottomID,
        Boolean flywheelTopInverted,
        Boolean flywheelBottomInverted,
        Range flywheelTopMinMax,
        Range flywheelBottomMinMax,
        PIDConstants flywheelTopPID,
        PIDConstants flywheelBottomPID,
        FFConstants flywheelTopFF,
        FFConstants flywheelBottomFF) {
      this.flywheelTopID = flywheelTopID;
      this.flywheelBottomID = flywheelBottomID;
      this.flywheelTopInverted = flywheelTopInverted;
      this.flywheelBottomInverted = flywheelBottomInverted;
      this.flywheelTopMinMax = flywheelTopMinMax;
      this.flywheelBottomMinMax = flywheelBottomMinMax;
      this.flywheelTopPID = flywheelTopPID;
      this.flywheelBottomPID = flywheelBottomPID;
      this.flywheelTopFF = flywheelTopFF;
      this.flywheelBottomFF = flywheelBottomFF;
    }
  }
}
