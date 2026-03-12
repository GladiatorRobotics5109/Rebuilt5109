package frc.robot.subsystems.hood;

import static frc.robot.Constants.HoodConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Conversions;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX m_motor;

    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

    private final StatusSignal<Angle> m_position;
    private final StatusSignal<AngularVelocity> m_velocity;
    private final StatusSignal<Voltage> m_appliedVolts;
    private final StatusSignal<Current> m_statorCurrent;
    private final StatusSignal<Current> m_supplyCurrent;
    private final StatusSignal<Temperature> m_temp;

    private final Debouncer m_connectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    private final TalonFXConfiguration m_config = new TalonFXConfiguration();

    public HoodIOTalonFX(int id, CANBus canbus) {
        m_motor = new TalonFX(id, canbus);

        m_config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        m_config.CurrentLimits.StatorCurrentLimitEnable = kStatorCurrentLimitEnable;

        m_config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        m_config.CurrentLimits.SupplyCurrentLimitEnable = kSupplyCurrentLimitEnable;

        m_config.Feedback.SensorToMechanismRatio = kGearRatio;

        m_config.MotorOutput.Inverted = kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        m_config.Slot0.kP = kP;
        m_config.Slot0.kI = kI;
        m_config.Slot0.kD = kD;

        m_config.Slot0.kS = kS;
        m_config.Slot0.kV = kV;
        m_config.Slot0.kA = kA;
        m_config.Slot0.kG = kG;

        m_config.MotionMagic.MotionMagicCruiseVelocity = Conversions.radiansToRotations(
            kMotionMagicCruiseVelocityRadPerSec
        );
        m_config.MotionMagic.MotionMagicAcceleration = Conversions.radiansToRotations(
            kMotionMagicCruiseAccelerationRadPerSecSq
        );
        
        m_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxAngle.getRotations();
        m_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinAngle.getRotations();
        m_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        StatusCode result = m_motor.getConfigurator().apply(m_config);
        if (!result.isOK()) {
            DriverStation.reportWarning(
                "Failed to apply hood configs!\nName: "
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
    public void updateInputs(HoodIOInputs inputs) {
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
    public void runPosition(double positionRad) {
        m_motor.setControl(m_motionMagic.withPosition(Units.Radians.of(positionRad)));
    }

    @Override
    public void setPID(double p, double i, double d) {
        m_config.Slot0.kP = kP;
        m_config.Slot0.kI = kI;
        m_config.Slot0.kD = kD;

        StatusCode result = m_motor.getConfigurator().apply(m_config);
        if (!result.isOK()) {
            DriverStation.reportWarning(
                "Failed to apply hood configs!\nName: "
                    + result.getName()
                    + "\nDescription: "
                    + result.getDescription(),
                true
            );
        }
    }

    @Override
    public void setFF(double s, double v, double a, double g) {
        m_config.Slot0.kS = s;
        m_config.Slot0.kV = v;
        m_config.Slot0.kA = a;
        m_config.Slot0.kG = g;

        StatusCode result = m_motor.getConfigurator().apply(m_config);
        if (!result.isOK()) {
            DriverStation.reportWarning(
                "Failed to apply hood configs!\nName: "
                    + result.getName()
                    + "\nDescription: "
                    + result.getDescription(),
                true
            );
        }
    }

    @Override
    public void setMotionMagic(double cruiseVelocityRadPerSec, double accelerationRadPerSecSq) {
        m_config.MotionMagic.MotionMagicCruiseVelocity = Conversions.radiansToRotations(cruiseVelocityRadPerSec);
        m_config.MotionMagic.MotionMagicAcceleration = Conversions.radiansToRotations(accelerationRadPerSecSq);

        StatusCode result = m_motor.getConfigurator().apply(m_config);
        if (!result.isOK()) {
            DriverStation.reportWarning(
                "Failed to apply hood configs!\nName: "
                    + result.getName()
                    + "\nDescription: "
                    + result.getDescription(),
                true
            );
        }
    }
}
