# Adding feedforward to the Swerve modules

## The importance of feedforward
Because the wheels on a Swerve Drive chassis all run at their own speeds, its very important that they run at the precise speed commanded to them, or else the robot will behave in an uncontrolled manner. This can be accomplished with feedback control, where the motor controller (Spark MAX in our case) compares the velocity setpoint to the measured speed, and then adjusts the voltage to make them match. The feedback algorithm that Spark MAXes use is [PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html).

Regular PID doesn't work well for velocity, since you need some non-zero voltage to maintain a velocity setpoint, but the error will be at 0, demanding no voltage. Currently we compensate for this by using a basic feedforward term. The feedforward term `kV` (or sometimes referred to as `kF`) is multiplied by the setpoint, so the voltage scales linearly with the velocity.

The downside of this however, is that the equation for a motor isn't simply `V = kV * v`, where `V` is the output voltage, and `v` is the velocity setpoint. The [real equation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation) is `V = kS + kV * v + kA * a`, where `kS` is the voltage required to overcome and static forces, such as friction, and `a` is the instantaneous acceleration of the motor. Sometimes the acceletion term `kA * a` is ignored when there is no acceleration. A flywheel, for instance, has no acceleration when its velocity is at its setpoint, but `kS` and `kV` are still active to overcome static forces and hold the flywheel at its setpoint.

### Why is kS important?
The importance of `kS` can be demonstrated pretty easily.

Imagine a motor that had constants `kS = .5` and `kV = .5`, and was not accelerating. The equation for required voltage would be

```
V = .5 + .5*v
```

If you wanted to run the motor at a speed of `1`, then the required voltage would be 

```
V = .5 + .5*1 = 1
```

Lets try removing `kS` from the equation and try to calclate `kV` using the measured voltage of `1` and the setpoint of `1`
```
V = kV * v
kV = V / v
kV = 1 / 1
kV = 1
```
 Uh oh. Our kV was calculated to be `1`, instead of `.5`. This will work fine when `v=1`, but what if we wanted to find the voltage required to run the motor at a speed of `5`

```
V = kV * v
V = 1 * 5
V = 5
```

Compared with using the correct value of `kS` in the equation

```
V = kS + kV * v
V = .5 + .5 * 5
V = .5 + 2.5
V = 3
```

This means if we ignore `kS`, then our `kV` will be too strong and produce too high a voltage at higher speeds. It will also be too weak at lower speeds.


## Adding feedforward to our motors

Unfortunately, Spark MAX motor controllers don't allow setting `kS`, `kV`, and `kA`. They only allow setting a single `kF`, which doesn't work well for multiple velocity setpoints as we showed previously. 

Instead, we can calcualte the feedforward voltage within our robot code, and send it to the motor controller as _arbitrary feedforward_. Arbitrary means the motor controller will simply add it on to the voltage calculated via the PID loop, rather than multiplying it by the velocity setpoint. In mathematical terms:

```
V = feedbackVoltage + arbitraryFeedforward
```
vs
```
V = feedbackVoltage + feedforward * setpoint
```

Our arbitrary feedforward _will_ account for the setpoint in its calculation.

#### WPILib FeedForward class
WPILib has a helper class that accepts our feedforward constants `kS`, `kV`, and `kA` and calculates `V` based on the requested velocity and current velocity. The current velocity is used to calculate acceleration for the `kA` term.


### How will it work

1. The `Drive` command tells the `Chassis` subsystem how fast to drive the entire robot
2. `Chassis` converts the robot speed into individual swerve module speeds and sends the individual speeds to the modules
3. The `NeoModule` calculates the predetermined voltage using a `SimpleMotorFeedforward` object, and commands its `driveMotor` to run at the speed requested, and suggests the feedforward voltage

## Add feedforward parameters to `ClosedLoopParameters`

Currently `ModuleConfig.ClosedLoopParameters` only supports `kP`, `kI`, `kD`, and `kF`. Let's modify the class to add `kS`, `kV`, and `kA`.

```java
public static class ClosedLoopParameters {
    public double kP, kI, kD, kF, kS, kV, kA;

    public ClosedLoopParameters(double kP, double kI, double kD, double kF, double kS, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
}
```

## Creating a SimpleMotorFeedforward object

In the `NeoModule` class, _declare_ a private member variable of type `SimpleMotorFeedforward`

```java
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class NeoModule {
    private final SimpleMotorFeedforward;
}
```

We do not assign a value to the variable yet, because in order to construct a `SimpleMotorFeedforward` we need our constants, and those are passed into the constructor. 

In the constructor:
```java
public NeoModule(ModuleConfig config) {
    this.driveFeedforward = new SimpleMotorFeedforward(
        config.driveClosedLoopParameters.kS,
        config.driveClosedLoopParameters.kV,
        config.driveClosedLoopParameters.kA);
}
```


## Calculating the feedforward

Using the `SimpleMotorFeedforward` is very easy. Whenever we are given a new speed to drive the module at, we pass that speed into the feedforward object.

For a `NeoModule`, the method that is called to request a new state is `setState`. Here we will use the current velocity of the drive motor along with the requested speed to calculate feedforward.

After calculating the `optimizedState` of the swerve module, get the current velocity and calculate the feedforward.

```java
public void setState(SwerveModuleState state) {
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getSteerPosition());

    double currentVelocity = driveEnc.getVelocity();
    // 20ms (50Hz) is the default period (frequency) of the roboRIO control loop
    double feedForward = driveFeedforward.calculate(currentVelocity, optimizedState.speedMetersPerSecond, 0.02);

...
}
```

## Sending arbitrary feedforward to the Spark MAX

Currently the method used to control the drive motor is
```java
public void setState(SwerveModuleState state) {
    ...
    drivePID.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    ...
}
```

We will use [an alternate version of this method](https://codedocs.revrobotics.com/java/com/revrobotics/sparkpidcontroller#setReference(double,com.revrobotics.CANSparkBase.ControlType,int,double)) that accepts two more paramters, a `pidSlot` (which can be used to switch between stored PID parameters), and an `arbFeedforward`. Adjust the existing code to add in these arguments
```java
public void setState(SwerveModuleState state) {
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getSteerPosition());

    double currentVelocity = driveEnc.getVelocity();
    // 20ms (50Hz) is the default period (frequency) of the roboRIO control loop
    double feedForward = driveFeedforward.calculate(currentVelocity, optimizedState.speedMetersPerSecond, 0.02);

    drivePID.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, feedForward);
    ...
}
```

## Configure the motor feedforward constants

The `ModuleConfig` object is created in the `*RobotContainer` classes. We need to plug in the feedforward constants retrieved by the SysID routine Emma ran on the robot. 

In `CompetitionRobotContainer`, adjust the `makeSwerveModule` method to take in `kS`, `kV`, and `kA` arguments, and pass them into the `driveClosedLoopParams` object

```java
  private NeoModule makeSwerveModule(int driveId, int steerId, double kS, double kV, double kA) {
    ModuleConfig.ClosedLoopParameters driveClosedLoopParams =
        new ModuleConfig.ClosedLoopParameters(0.1, 0, 0, 1, kS, kV, kA);
    ModuleConfig.ClosedLoopParameters steerClosedLoopParams =
        new ModuleConfig.ClosedLoopParameters(18, 0, 0, 0, 0, 0, 0);
```

The steer motor does not use feedforward, so we set the feedforward values to `0`

In the constructor of `CompetitionRobotContainer`, we can pass in the feedforward params from [the spreadsheet](https://docs.google.com/spreadsheets/d/1s1rihssGtqzc88wtcjM1EgSCxngTgr22rPdJCgehazA/edit#gid=0). 

> Replace `kS`, `kV`, and `kA` with the actual values from the spreadsheet. The first number passed to `makeSwerveModule` corresponds to the motor number in the spreadsheet
```java
  CompetitionRobotContainer() {

    NeoModule frontLeft = makeSwerveModule(1, 2, kS, kV, kA);
    NeoModule frontRight = makeSwerveModule(3, 4, kS, kV, kA);
    NeoModule backLeft = makeSwerveModule(5, 6, kS, kV, kA);
    NeoModule backRight = makeSwerveModule(7, 8, kS, kV, kA);
```

## Test the values

Put the robot on the ground and drive around. It should drive in straight lines better now that the motors can more easily drive at their requested speeds