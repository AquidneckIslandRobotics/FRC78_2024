# Elevator Zeroing Routine

## The Setup
The elevator is a linear mechanism that raises the shooter up and down, and will eventually hold the hooks to climb during the endgame.

The elevator is driven by two chains that are screwed into the elevator with custom mounting blocks. As the chain is turned by the gearbox, the chain pulls on the mounting block and elevator, moving it up and down.

The elevator postition is known by a through-bore encoder attached to the bottom sprocket axle. We chose an external encoder rather than using the built-in NEO encoders becuase there are many gears between the motor and the elevator, and with every gear connection there is additional slop due to backlash. Backlash is why you can wiggle the output shaft without causing the motors to move. There is a small amount of play between the teeth of the gears, and this slop multiplies over many gears.

## Why do we have to zero?
Unlike the encoder that is used for the swerve drive module steer angle, the encoder on the elevator is _not_ and absolute encoder. Absolute encoders work very well when the mechanism you're measuring only rotates a maximum of 1 time (or wraps around after 1 rotation, like the swerve wheels). The elevator sprocket axle will rotate a little under 3 times as the elevator moves from the bottom of its range to the top. An absolute encoder would not be able to handle rotating more than once.

Because the encoder passes through the same spot in its rotation 3 times as the elevator moves, there's no way to know which spot it's in when the robot turns on. Instead, we need to move the elevator to a known 'zero' location, reset the encoder value, and then measure from that known location. 

## How do we know when we're at the zero spot?
Limit switches! Limit switches are electrical components that tell you when a mechanism is in a specific spot. They can be physical switches like buttons, light based switches like a beam break sensor, or magnetic switches like the Hall-Effect sensor we're using. 

The Hall-Effect sensor works by detecting when a magnet passes over an electronic circuit. By attaching the magnet to the elevator and the sensor to the frame, we can detect when the elevator moves to a specific position. 

Limit switches can do more than just help you zero the encoders for a mechanism. Motor controllers in FRC often let you hook up limit switches to prevent the mechanism from moving past a certain point, helping prevent damage to the mechanism or robot frame.

On the current robot, there is a single limit switch mounted at the bottom of the elevator. When the elevator is in its bottom-most position the limit switch will be triggered and the elevator will be prevented from moving further.

## A Slight Problem
Because we have two motors connected together through a gearbox, we always want them to run at the exact same speed all the time. To do this, we put one of the motors in 'leader' mode, and one of the motors in 'follower' mode. The following motor will always output the same duty-cycle as the leader.

However, the Spark MAX motor controllers do not allow connecting limit switches and non-absolute encoders at the same time. This makes it hard to have both hard and soft limits for our mechanism. One motor has the hard, physical limit switch, and the other will have the soft, encoder-based limit. 

To get around this, we will change which motor is the leader and follower after zeroing. Once we know where the elevator is, it isn't all that important to have the hard limit switch. We can just use the encoder reading to find a soft limit.

## The Zeroing Routine

1. Set the motor with the limit switch as the leader, and the other as the follower
2. Slowly lower the elevator until the hard limit has been reached
3. Reset the relative encoder to 0
4. Disable the hard limit on the leader motor.
5. Set the motor with the encoder to the leader, and the motor with the limit switch to follow
6. Set the soft limits on the new leader based on encoder positions
7. Mark the elevator as 'zeroed'
7. Set this routine to run whenever the elevator is not 'zeroed'

## Let's make it happen!
Most of these changes will take place in `Elevator.java`. The structure of the solution will group steps of the zeroing process into methods that are commands. There will be one main zeroing method that links all the steps in order.

### Zeroed variable
Begin by adding a **private** boolean `zeroed` and a getter method to access the value
```java
  private boolean zeroed = false;

  public boolean hasNotBeenZeroed() {
    return !zeroed;
  }
```

We want the variable to be private because we only want to be able to change its value from within this class. No joystick button or command should be able to mark the elevator as zeroed, but they _can_ check if the elevator has been zeroed or not

The reason why the method returns `true` if the elevator is _not_ zeroed is so we can use this method to trigger the zeroing command later on, as well as disable other mechanisms if the elevator isn't zeroed.

## Limit Switch and Encoder
Create a `SparkLimitSwitch` object that holds the reverse limit switch from motor 12, and move the `RelativeEncoder` object out of the constructor
```java
/** The limit switch at the bottom of the elevator */
private final SparkLimitSwitch reverseLimitSwitch;
private final RelativeEncoder encoder;

public Elevator() {
    reverseLimitSwitch = elevNeoMotor2.getReverseLimitSwitch(Type.kNormallyOpen);
    encoder = elevNeoMotor1.getAlternateEncoder(8192);
}
```

### Configure motors for Zeroing
To configure the motors for zeroing, we will set motor 11 to follow motor 12, enable the limit switch on motor 12, and set the speed to 0 to halt any motion

```java
  private Command configureMotorsForZeroing() {
    return runOnce(
        () -> {
          elevNeoMotor1.follow(elevNeoMotor2, true);
          reverseLimitSwitch.enableLimitSwitch(false);
          elevNeoMotor2.set(0);
        });
  }
```
Because this config only needs to happen once, we can use the `runOnce` factory.

### Slowly lower the elevator
Once the motors are configured, we can begin slowly lowering the elevator until the limit switch is reached

```java

  private Command lowerElevatorUntilLimitReached() {
    return startEnd(() -> elevNeoMotor2.set(-.1), () -> elevNeoMotor2.set(0))
        .until(reverseLimitSwitch::isPressed);
  }
```

The until condition will continuously check for the reverse limit switch to be tripped. Once it is, the elevator will stop moving, and we can reset the encoder position

### Configure motors after zeroing
Now that the elevator is in a known good spot, we can zero the encoder, disable the hard limit switch, swap the leader and follower, and enable soft limits. 

The reason we want to disable the hard limit is because our leader motor will not know if the limit switch has been tripped. If something goes wrong and the hard limit is tripped accidentally, we don't want our leader motor to continue running and the follower motor to stop. Instead, we'll trust the encoder position to stop the motor from moving past its limits.

```java
  private Command configureMotorsAfterZeroing() {
    return runOnce(
      () -> {
          // Reset encoder
          encoder.setPosition(0);
          // Swap follower and stop the new leader
          elevNeoMotor2.follow(elevNeoMotor1);
          elevNeoMotor1.set(0);
          // Disable hard limit switch
          reverseLimitSwitch.enableLimitSwitch(false);
          // Add a soft limit to keep elevator 1 inch from the top and bottom
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kForward, 14);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kReverse, 1);
          // Mark elevator as zeroed
          zeroed = true;
      });
  }
```

### Putting it all together
We can use the `andThen` command factory to link our steps together in a `zeroEncoder` command method

```java
  /** Runs sequence for zeroing elevator based off magnetic limit switch */
  public Command zeroElevator() {
    return configureMotorsForZeroing()
        .andThen(lowerElevatorUntilLimitReached())
        .andThen(configureMotorsAfterZeroing())
        // Do not allow interrupting the zeroing behavior!
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
```

Note that this is the only **public** method that deals with zeroing the elevator, since it doesn't make sense for someone to just call one of the other methods without running the entire sequence.

The last line tells the command scheduler to cancel any command that might use the elevator subsystem while the zeroing command is active. We never want to interrupt the zeroing behavior, because all functionality of the elevator will rely on it being properly zeroed.

### Zeroing the elevator if it isn't zeroed
Remember the `hasNotBeenZeroed` method we created earlier? Now we'll bind that method to a trigger that runs the zeroing command at the beginning of the robot being enabled.

Let's head over to the `CompetitionRobotContainer` class, in the `configureBindings` method. Here we'll add a trigger that runs the zeroElevator command as long as the elevator has not been zeroed, and the driver station is enabled
```java
public void configureBindings() {
    new Trigger(m_Elevator::hasNotBeenZeroed)
        .and(DriverStation::isEnabled)
        .whileTrue(m_Elevator.zeroElevator());
}
```

Once you enable the robot, the `zeroElevator` command will run, slowly lowering the elevator until the limit switch is pressed. We will need some safety precautions added in, such as current sensing to avoid burning out the motors in the event the limit switch
## DANGER
With the addition of this code, the robot has become semi-autonomous. Now, as long as there has been a power cycle or other reset of the robot code (such as deploying), enabling the robot will trigger the zeroing routine. You must be very careful that the robot is clear of hands and everyone nearby is wearing proper safety protection when enabling the robot


### Future work
We have a way of running the zeroing command while the elevator, but we can use the `hasNotBeenZeroed` method to prevent _other_ mechanisms from running as well. For example, perhaps we don't want the wrist mechanism to move while we're zeroing. We can create an uninterruptible command that stops the motor, and have that command run while the `hasNotBeenZeroed` method returns true. It might look like

```java
public class Wrist {
    public Command disable() {
        return run(
            () -> 
                motor.set(0)
                .withInterruptBehavior(
                        InterruptionBehavior.kCancelIncoming
                )
            )
    }
}

// In CompetitionRobotContainer
public configureBindings() {
    new Trigger(m_Elevator::hasNotBeenZeroed)
        .whileTrue(wrist.disable());
}
```