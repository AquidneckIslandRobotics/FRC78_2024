# Characterizing a Drivetrain with SysId

This tutorial will be tailored to the 2024 robot and codebase. A more generalized overview can be
found [on the FRC Docs website](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html)

## Robot Characterization

It's very common to use PID control to precisely articulate robot mechanisms. PID only uses the
error between the robot's current state and its target state. This is a lot of work for PID to
accomplish on its own, and we can use our knowledge of DC motors to provide a better estimate of the
output required for the motor to move the way we want.

Using models of the robot system to help determine how our motor will behave is called 'feedforward
control'. Compared with feedback control like PID, feedforward control does not use any sensor data
in its calculations. Paired together, feedforward and feedback can provide very fast and accurate
control of your robot.

## The Equation We Need to Solve

The feedforward equation that SysId will help us solve is
the [Permanent-Magnet DC Motor Feedforward Equation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation).
There are variants of this equation that also account for gravity, such as in an elevator or arm.
Gravity does not affect the drivetrain, so we will use the basic form of the equation.

## Overview of the Solution

At the end of this tutorial, you will have created

- A SysIdRoutine object
    - This will tell WPILib how to use our `Chassis` class to drive in a controlled manner
- A method to send voltage to the 4 drive motors, while commanding the steer motors to
  keep the wheels pointed straight ahead
- Logging for the SysId
- 2 Command Factory methods for the quasistatic and dynamic tests
    - These methods will link to the operator controller to run the SysId routine
- 4 button bindings to run each of the 4 tests that are required for characterization

Each step in this tutorial is linked to the FRC Docs page that covers the necessary info. If you'd
like to try figuring out how to adapt the docs to our specific code, click on each header to find
the corresponding article.

## [Create the SysIdRoutine](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html)

Let's begin in the `Chassis` class. The `Chassis` class represents the drivetrain as a whole.
Commands are used to request specific speeds from the robot, and the `Chassis` class performs the
necessary operations to determine the speeds of each `SwerveModule`.

### Create a `SysIdRoutine` object

At the top of the class, before the constructor, create a new `SydIdRoutine` instance
named `drivetrainRoutine`.

```java
public class Chassis extends SubsystemBase {

  private SysIdRoutine drivetrainRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));
}
```

The `::` syntax might look new to you. This represents a reference to a method, without calling the
method. This allows the `SysIdRoutine.Mechanism` know which methods to call in order to provide
output to the motors (via `voltageDrive`) and save the data in the logs (via `logMotors`). VSCode
might give errors at this point because those methods don't exist. Let's create them now.

### Add routine methods

[Using the Javadocs](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/sysid/SysIdRoutine.Mechanism.html#%3Cinit%3E(java.util.function.Consumer,java.util.function.Consumer,edu.wpi.first.wpilibj2.command.Subsystem))
we can see that the first argument of the `Mechanism` constructor is of
type `Consumer<Measure<Voltage>>`, and the second argument is of
type `Consumer<SysIdRoutineLog>`. `Consumer` is a fancy Java class that means a method that has an
argument of type ___. In step 1 we passed in `this::voltageDrive` as the first argument, so that
method should have 1 parameter of type `Measure<Voltage>`.
At the bottom of the `Chassis` class, create the `voltageDrive` method, `

```java
/* Called by the SysIdRoutine */
private void voltageDrive(Measure<Voltage> voltage) {
}
```

We'll do the same for `logMotors`, which has 1 parameter of type `SysIdRoutineLog`.

```java
/* Called by the SysId routine */
private void logMotors(SysIdRoutineLog log) {
}
```

### Command the swerve modules in `voltageDrive`

`VoltageDrive` will be called perioodically while the SysId routine is running. This method needs to
send the requested voltage to the drive motors of each swerve module.

```java
public void voltageDrive(Measure<Voltage> voltage) {
  // Iterate through each of the 4 swerve modules
  for (SwerveModule module : modules) {
    // Call the open loop method, which sends a voltage with no feedback to the motors, but holds the wheel at 0 degrees
    module.openLoopDiffDrive(voltage.in(Volts));
  }
}
```

### Create openLoopDiffDrive and logMotor methods

The `openLoopDiffDrive` method called in the previous step doesn't exist yet, since up until now we
have only used closed-loop control for the swerve drive. We need to add the method to
the `SwerveModule` interface, as well as any implementations of that interface (currently
only `NeoModule`).

The `logMotor` method will also be called on each module to log its feedback data. We'll add that to
the SwerveModule interface as well

Add the `openLoopDiffDrive` method to `SwerveModule.java`

```java
public interface SwerveModule {

  /** Runs the drive motor at a set voltage, while keeping the steer angle at 0 degrees */
  void openLoopDiffDrive(double voltage);

  /** Logs the motor position, velocity, and voltage data for SysId */
  void logMotor(SysIdRoutineLog log);
}
```

The implementation in `NeoModule` will set the reference point of the steer module to 0 rotations,
which corresponds to straight ahead. It will also set the voltage of the drive motor to the
requested voltage

```java
// In NeoModule.java
public void openLoopDiffDrive(double voltage) {
  steerPID.setReference(0, ControlType.kPosition);
  drive.setVoltage(voltage);
}
```

### Log the motor values to the SysIdRoutineLog

The log file collects data from the motors so that the SysId tool can analyze the effect of Voltage
on the motor output.
Back in `Chassis.java`, we can fill in the `logMotor` method.

```java
public void logMotor(SysIdRoutineLog log) {
  for (SwerveModule module : modules) {
    // Each motor will write to the log directly
    module.logMotor(log);
  }
}
```

In `NeoModule` we can add the code for the `logMotor` method.

To avoid creating new `Measure` objects every time the `logMotors` method is called, we can use a
special version of the `Measure` class that allows mutating the held value. This helps to avoid
running out of memory on the RIO.

```java
/* Mutate these each time we log so that we aren't creating objects constantly */
private final MutableMeasure<Voltage> mutableAppliedVoltage = mutable(Volts.of(0));
private final MutableMeasure<Distance> mutableDistance = mutable(Meters.of(0));
private final MutableMeasure<Velocity<Distance>> mutableVelocity = mutable(MetersPerSecond.of(0));

public void logMotor(SysIdRoutineLog log) {
  log.motor("motor#" + config.driveID)
     // Log voltage
     .voltage(
         mutableAppliedVoltage.mut_replace(
             // getAppliedOutput return the duty cycle which is from [-1, +1]. We multiply this
             // by the voltage going into the spark max, called the bus voltage to receive the
             // output voltage
             drive.getAppliedOutput() * drive.getBusVoltage(), Volts))
     // the drive encoder has the necessary position and velocity conversion factors already set
     .linearVelocity(mutableVelocity.mut_replace(driveEnc.getVelocity(), MetersPerSecond))
     .linearPosition(mutableDistance.mut_replace(driveEnc.getPosition(), Meters));
}
```

### [Creating command factories to run the routine](https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html#included-command-types)

Now it's time to add the `Command`s that will run the SysId routines. The command factories will
live in the `Chassis` class, since they're very tightly associated with the `Chassis` subsystem.

```java
public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return routine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  return routine.dynamic(direction);
}
```

## Bind the routine to operator controller buttons

We're almost there! Now we just need to add triggers from our Xbox Controller to run the routine
commands.

We'll bind the forward tests to A and B, and the backwards tests to X and Y.
In `CompetitionRobotContainer.java` `configureBindings` method:

```java
private void configureBindings() {
  // The routine automatically stops the motors at the end of the command
  new Trigger(m_driveController::getAButton).whileTrue(
      m_chassis.sysIdQuasistatic(Direction.kForward));
  new Trigger(m_driveController::getBButton).whileTrue(m_chassis.sysIdDynamic(Direction.kForward));
  new Trigger(m_driveController::getXButton).whileTrue(
      m_chassis.sysIdQuasistatic(Direction.kReverse));
  new Trigger(m_driveController::getYButton).whileTrue(m_chassis.sysIdDynamic(Direction.kReverse));
}
```

All done! Now it's time to deploy and test your code on the real robot :) 