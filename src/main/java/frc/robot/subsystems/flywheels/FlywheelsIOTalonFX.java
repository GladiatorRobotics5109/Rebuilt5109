package frc.robot.subsystems.flywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.FlywheelsConstants.*;

public class FlywheelsIOTalonFX implements FlywheelsIO {
    private final TalonFX m_motor;

    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0.0);
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);

    private final StatusSignal<Angle> m_position;
    private final StatusSignal<AngularVelocity> m_velocity;
    private final StatusSignal<Voltage> m_appliedVolts;
    private final StatusSignal<Current> m_statorCurrent;
    private final StatusSignal<Current> m_supplyCurrent;
    private final StatusSignal<Temperature> m_temp;

    private final Debouncer m_connectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    private TalonFXConfiguration m_config = new TalonFXConfiguration();

    public FlywheelsIOTalonFX(int id, CANBus canbus) {
        m_motor = new TalonFX(id, canbus);

        m_config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        m_config.CurrentLimits.StatorCurrentLimitEnable = kStatorCurrentLimitEnable;

        m_config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        m_config.CurrentLimits.SupplyCurrentLimitEnable = kSupplyCurrentLimitEnable;

        m_config.Slot0.kP = kP;
        m_config.Slot0.kI = kI;
        m_config.Slot0.kD = kD;

        m_config.Feedback.SensorToMechanismRatio = kGearRatio;

        m_config.MotorOutput.Inverted = kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        StatusCode result = m_motor.getConfigurator().apply(m_config);
        if (!result.isOK()) {
            DriverStation.reportWarning(
                "Failed to apply flywheels configs!\nName: "
                    + result.getName()
                    + "\nDescription: "
                    + result.getDescription(),
                true
            );
        }

        m_position = m_motor.getPosition();
        m_velocity = m_motor.getVelocity();
        m_appliedVolts = m_motor.getMotorVoltage();
        m_statorCurrent = m_motor.getStatorCurrent();
        m_supplyCurrent = m_motor.getSupplyCurrent();
        m_temp = m_motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_position,
            m_velocity,
            m_appliedVolts,
            m_statorCurrent,
            m_supplyCurrent
        );

        ParentDevice.optimizeBusUtilizationForAll(m_motor);
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        StatusCode status = BaseStatusSignal.refreshAll(
            m_position,
            m_velocity,
            m_appliedVolts,
            m_statorCurrent,
            m_statorCurrent,
            m_temp
        );

        inputs.connected = m_connectedDebounce.calculate(status.isOK());
        inputs.positionRad = m_position.getValue().in(Units.Radians);
        inputs.velocityRadPerSec = m_velocity.getValue().in(Units.RadiansPerSecond);
        inputs.appliedVolts = m_appliedVolts.getValue().in(Units.Volts);
        inputs.statorCurrentAmps = m_statorCurrent.getValue().in(Units.Amps);
        inputs.supplyCurrentAmps = m_supplyCurrent.getValue().in(Units.Amp);
        inputs.tempCelsius = m_temp.getValue().in(Units.Celsius);
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        m_motor.setControl(m_velocityVoltage.withVelocity(velocityRadPerSec));
    }

    @Override
    public void setPID(double p, double i, double d) {
        m_config.Slot0.kP = kP;
        m_config.Slot0.kI = kI;
        m_config.Slot0.kD = kD;

        StatusCode result = m_motor.getConfigurator().apply(m_config);
        if (!result.isOK()) {
            DriverStation.reportWarning(
                "Failed to apply flywheels configs!\nName: "
                    + result.getName()
                    + "\nDescription: "
                    + result.getDescription(),
                true
            );
        }
    }
}
