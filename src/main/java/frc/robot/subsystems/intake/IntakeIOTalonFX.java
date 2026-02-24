package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;


public abstract class IntakeIOTalonFX implements IntakeIO {
    @Override
    public void updateInputsPivot(IntakeIOInputsPivot inputs) {  
    }   
    @Override
    public void rollersRunVoltage(double volts) {
    }
    @Override
    public void pivotRunVoltage(double volts) {
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // implement reading/updating both TalonFX inputs here
    }
    @Override
    public void pivotRunPosition(Rotation2d position) {
    }
}
//Kraken X44 
//X60
 

 // why no override??-- gives error if I put override, but it is still overriding the default methods in the interface. I think it’s because the default methods in the interface have different names than the methods in the TalonFX implementation, so they are not technically overriding each other. The methods in the TalonFX implementation are just implementing the interface methods, but they are not marked with @Override because they have different names.

/*Implement the IntakeIO interface for a TalonFX
Don’t worry about configuring the TalonFXs, I can do that later
@Override public void updateInputs(IntakeIOInputs inputs) - Updates the inputs from both TalonFXs
@Override public void rollersRunVoltage(double volts) - Runs the intake rollers with the provided voltage (positive = intake into the bot, negative = reverse)
@Override public void pivotRunVoltage(double volts) - Runs the intake pivot with the provided voltage
@Override public void pivotRunPosition(Rotation2d position) - Runs the intake pivot to the provided position using position PID on the TalonFX*/
