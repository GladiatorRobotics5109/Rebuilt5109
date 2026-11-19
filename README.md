# Rebuilt 5109
5109's 2026 FRC Robot Code

# Intake Subsystem Description
The intake subsystem consists of 2 motors:
- 1 Kraken x60 on a 90:1 gearbox that controls the pivot of the intake.
- 1 Kraken x44 on a 3:1 gearbox that controls the rolls of the intake.

The intake must be able to be deployed and stowed.

Deploying the intake consists of: 
- Setting the position of the pivot to ```IntakeConstants.kPivotDeployedPosition```
- Setting the rollers voltage to ```IntakeConstants.kRollersIntakeVoltage```

Stowing the intake consists of:
- Setting the position of the pivot to ```IntakeConstants.kPivotStowedPosition```
- Setting the rollers voltage to ```0```

# Task 1
Reimplement:
- [IntakeSubsystem.java](src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java)
- [IntakeIO.java](src/main/java/frc/robot/subsystems/intake/IntakeIO.java)

Don't worry about adding logging support or writing the IO implementations for now.

I would recommend starting with [IntakeIO.java](src/main/java/frc/robot/subsystems/intake/IntakeIO.java) and then implementing [IntakeSubsystem.java](src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java).
## File Structure
### [IntakeSubsystem.java](src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java)
Main subsystem class, controls high-level subsystem actions.
For example, the intake needs to be deployed and stowed. The ```runPivotVoltage``` and ```runRollersVoltage``` are only really used to testing / debugging.
#### Basic Outline
```java
public class IntakeSubsystem extends SubsystemBase {
    // Hold a reference to the IO implementation of this subsystem
    private final IntakeIO m_io;

    public IntakeSubsystem() {
        // Initialize IO depending on the current robot mode.
        m_io = switch (Constants.kCurrentMode) {
            case REAL -> new IntakeIOTalonFX(kRollersId, kPivotId, Constants.kCANBusRio);
            case SIM -> new IntakeIOSim();
            default -> new IntakeIO() {};
        };
    }

    /**
     * Deploys the intake and starts the rollers.
     */
    public void deploy() {}

    /**
     * Brings the intake back up and stops the rollers.
     */
    public void stow() {}

    /**
     * Stops both the pivot and rolls of the intake.
     */
    public void stop() {}

    /**
     * Runs the intake pivot motor at the provided voltage.
     *
     * @param volts the voltage to run the motor at.
     */
    public void runPivotVoltage(double volts) {}

    /**
     * Runs the intake rollers at the provided voltage.
     *
     * @param volts the voltage to run the motor at.
     */
    public void runRollersVoltage(double volts) {}

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            stop();
        }
    }
}
```
### [IntakeIO.java](src/main/java/frc/robot/subsystems/intake/IntakeIO.java)
Interface that defines behaviors all IO classes must implement.
IO stands for Input/Output.
Most logged data that flows in and out of the robot goes through an IO interface.
#### Interfaces
An interface is basically a list of methods.
All classes that choose to implement and interface **must** implement all if its non-default methods.
Classes that implement an interface can optionally override default methods.
A method in an interface can be marked as ```default``` to provide a default implementation of that method.
#### Why we Need an Interface
Because we want the ability to run the robot in the in real life, in the simulator, and in replay, we must write different code to control each of these devices ([IntakeIOTalonFX.java](src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java) and [IntakeIOSim.java](src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java)).
The code for controlling a specific IO device is called an IO implementation.
It would be redundant to rewrite the entire [IntakeSubsystem](src/main/java/frc/robot/subsystems/intake/IntakeSubsystem.java) for each implementation, so we separate the code that handles controlling the specific IO implementation into its own class (Ex: creating the ```TalonFX``` object and configuring the motor).
We keep all the code that does not depend on the specific IO implementation in the subsystem class (Ex: calling ```runPivotPosition``` to tell the motor to go to a certain position).

In summary, the IntakeIO interface allows us to run the code in real life and sim because it provides a common set of methods we can call on ```IntakeIOTalonFX``` and ```IntakeIOSim```.
#### Basic Outline
```java
public interface IntakeIO {
    /**
     * This class defines all of the inputs that the IntakeIO will record.
     */
    @AutoLog
    public class IntakeIOInputs {
        // Example inputs
        public double pivotPositionRad;
        public double rollersPositionRad;
        // Add more inputs as you see fit, look at other XIOInputs classes for some ideas.
        // The more inputs you add, the more data will be logged. More data logged = more information for you to debug.
        // So you can add a bunch of inputs!.
    }

    // -- We want all classes that implement IntakeIO to have the following methods --

    // All IO interfaces need a method to update their inputs.
    default void updateInputs(IntakeIOInputs inputs) { /* Leave this method blank */ }

    default void runPivotVoltage(double volts) { /* Leave this method blank */ }

    default void runPivotPosition(Rotation2d position) { /* Leave this method blank */ }

    default void runRollersVoltage(double volts) { /* Leave this method blank */ }
}
```
### [IntakeIOTalonFX.java](src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java)
The IO implementation for the TalonFX motor controller (AKA the Kraken running on the real robot).
### [IntakeIOSim.java](src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java)
The IO implementation for the simulator. It
### [IntakeCommands.java](src/main/java/frc/robot/commands/IntakeCommands.java)
Defines the commands for the intake subsystem.

# WPILib Version
2026.2.1
