// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final TalonFX feedMotor;
  public TimeOfFlight feedSensor;

  /** Creates a new Feed. */
  public Feeder() {
    feedMotor = new TalonFX(16);
    feedSensor = new TimeOfFlight(17);
    feedSensor.setRangeOfInterest(10, 10, 11, 11);

    // If no commands are using this subsystem, stop the motor
    setDefaultCommand(run(() -> feedMotor.set(0)));
  }

  public Command runFeed() {
    return this.run(() -> feedMotor.set(0.15));
  }

  public Command reverseFeed() {
    return this.run(() -> feedMotor.set(-0.5));
  }

  public Command fire() {
    return this.run(() -> feedMotor.set(0.5));
  }

  public boolean isNoteQueued() {
    return feedSensor.getRange() <= 125;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("feedSensor", feedSensor.getRange());
  }
}
