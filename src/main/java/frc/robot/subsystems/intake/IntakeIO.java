package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
        @AutoLog
    class IntakeIOInputsPivot {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

     class IntakeIOInputsRoller {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputsPivot(IntakeIOInputsPivot inputs) {}

    default void setVoltagePivot(double volts) {}

    default void setVelocityPivot(double velocityRadPerSec) {}

    
    default void updateInputsRoller(IntakeIOInputsRoller inputs) {}

    default void setVoltageRoller(double volts) {}

    default void setVelocityRoller(double velocityRadPerSec) {}

  }


  /*@AutoLog public static class IntakeIOInputs - The inputs for the IntakeIO (FlywheelsIOInputs is a good reference)
default void rollersRunVoltage(double volts) 
default void pivotRunVoltage(double volts)
default void pivotRunPosition(Rotation2d position)
default void updateInputs(IntakeIOInputs inputs)*/


  

