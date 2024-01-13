package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

   CANSparkMax driveMotor;
   CANSparkMax steerMotor;

    public SwerveModule(int driveId, int steerId) {
        driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerId, MotorType.kBrushless);
    }


    public void drive(SwerveModuleState state) {
      driveMotor.getPIDController().setReference(state.speedMetersPerSecond, ControlType.kVelocity);
      steerMotor.getPIDController().setReference(state.angle.getRotations(), ControlType.kPosition);
    }
}
