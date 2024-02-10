// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private TalonFX shooterTOP;
  private TalonFX shooterBOTTOM;

  private final VelocityVoltage shooterTopVV;
  private final VelocityVoltage shooterBottomVV;

  private final ShooterConfig config;

  /** Creates a new Shooter. */
  public Shooter(ShooterConfig config) {
    this.config = config;
    shooterTOP = new TalonFX(config.FLYWHEEL_TOP_ID);
    shooterBOTTOM = new TalonFX(config.FLYWHEEL_BOTTOM_ID);

    shooterTOP.getConfigurator().apply(new TalonFXConfiguration());
    shooterBOTTOM.getConfigurator().apply(new TalonFXConfiguration());

    var shooterTopConfigs = new Slot0Configs();
    shooterTOP.setInverted(true);
    shooterTopConfigs.kS = config.FLYWHEEL_TOP_S;
    shooterTopConfigs.kV = config.FLYWHEEL_TOP_V;
    shooterTopConfigs.kP = config.FLYWHEEL_TOP_P;
    shooterTopConfigs.kI = config.FLYWHEEL_TOP_I;
    shooterTopConfigs.kD = config.FLYWHEEL_TOP_D;

    shooterTOP.getConfigurator().apply(shooterTopConfigs);

    var shooterBottomConfigs = new Slot0Configs();
    shooterBOTTOM.setInverted(true);
    shooterBottomConfigs.kS = config.FLYWHEEL_BOTTOM_S;
    shooterBottomConfigs.kV = config.FLYWHEEL_BOTTOM_V;
    shooterBottomConfigs.kP = config.FLYWHEEL_BOTTOM_P;
    shooterBottomConfigs.kI = config.FLYWHEEL_BOTTOM_I;
    shooterBottomConfigs.kD = config.FLYWHEEL_BOTTOM_D;

    shooterBOTTOM.getConfigurator().apply(shooterBottomConfigs);

    shooterTopVV = new VelocityVoltage(0).withSlot(0);
    shooterBottomVV = new VelocityVoltage(0).withSlot(0);
  }

  /**
   * @return true if motor velocity is 90% of its setpoint
   */
  public boolean upToSpeed() {
    return shooterTOP.getVelocity().getValueAsDouble() > shooterTopVV.Velocity * .9;
  }

  public void setPIDReferenceTOP(double setPoint) {
    shooterTOP.setControl(
        shooterTopVV.withVelocity(setPoint / 60).withFeedForward(config.FLYWHEEL_TOP_FF));
  }

  public void setPIDReferenceBOTTOM(double setPoint) {
    shooterBOTTOM.setControl(
        shooterBottomVV.withVelocity(setPoint / 60).withFeedForward(config.FLYWHEEL_BOTTOM_FF));
  }

  public void setPIDReferenceBOTH(double setPoint) {
    setPIDReferenceTOP(setPoint);
    setPIDReferenceBOTTOM(setPoint);
  }

  public void shooterSTOP() {
    shooterTOP.set(0);
    shooterBOTTOM.set(0);
  }

  public Command startShooter(int setPoint) {
    return this.runOnce(() -> this.setPIDReferenceBOTH(setPoint));
  }

  public Command stopCommand() {
    return this.runOnce(this::shooterSTOP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TOP SetPoint", shooterTOP.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("BOTTOM SetPoint", shooterBOTTOM.getVelocity().getValueAsDouble());
  }

  public static class ShooterConfig {
    // SHOOTER

    public int FLYWHEEL_TOP_ID;
    public int FLYWHEEL_BOTTOM_ID;

    // Constants - TOP FLYWHEEL
    public double FLYWHEEL_TOP_MIN;
    public double FLYWHEEL_TOP_MAX;
    // PID Consants - TOP FLYWHEEL
    public double FLYWHEEL_TOP_P;
    public double FLYWHEEL_TOP_I;
    public double FLYWHEEL_TOP_D;

    public double FLYWHEEL_TOP_S;
    public double FLYWHEEL_TOP_V;
    public double FLYWHEEL_TOP_FF;

    // Constants - BOTTOM FLYWHEEL
    public double FLYWHEEL_BOTTOM_MIN;
    public double FLYWHEEL_BOTTOM_MAX;
    // PID Constants - BOTTOM FLYWHEEL
    public double FLYWHEEL_BOTTOM_P;
    public double FLYWHEEL_BOTTOM_I;
    public double FLYWHEEL_BOTTOM_D;

    public double FLYWHEEL_BOTTOM_S;
    public double FLYWHEEL_BOTTOM_V;
    public double FLYWHEEL_BOTTOM_FF;

    /** Hood's angle of elevation in degrees */
    public double HOOD_ANGLE;
  }
}
