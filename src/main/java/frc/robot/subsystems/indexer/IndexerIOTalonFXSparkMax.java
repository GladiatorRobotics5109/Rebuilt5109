package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;
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

import static frc.robot.Constants.IndexerConstants.*;

public class IndexerIOTalonFXSparkMax implements IndexerIO {
    private final TalonFX m_indexer;

    private final VoltageOut m_indexerVoltageOut = new VoltageOut(0.0);

    private final StatusSignal<Angle> m_indexerPosition;
    private final StatusSignal<AngularVelocity> m_indexerVelocity;
    private final StatusSignal<Voltage> m_indexerAppliedVolts;
    private final StatusSignal<Current> m_indexerStatorCurrent;
    private final StatusSignal<Current> m_indexerSupplyCurrent;
    private final StatusSignal<Temperature> m_indexerTemp;

    private final Debouncer m_indexerConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    private TalonFXConfiguration m_indexerConfig = new TalonFXConfiguration();

    private final Debouncer m_kickupConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    private final SparkMax m_kickup;
    private final RelativeEncoder m_kickupEncoder;

    public IndexerIOTalonFXSparkMax(int indexerId, int kickupId, CANBus canbus) {
        m_indexer = new TalonFX(indexerId, canbus);

        m_indexerConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        m_indexerConfig.CurrentLimits.StatorCurrentLimitEnable = kStatorCurrentLimitEnable;

        m_indexerConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        m_indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = kSupplyCurrentLimitEnable;

        m_indexerConfig.Feedback.SensorToMechanismRatio = kIndexerGearRatio;

        m_indexerConfig.MotorOutput.Inverted = kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        StatusCode result = m_indexer.getConfigurator().apply(m_indexerConfig);
        if (!result.isOK()) {
            DriverStation.reportWarning(
                "Failed to apply indexer configs!\nName: "
                    + result.getName()
                    + "\nDescription: "
                    + result.getDescription(),
                true
            );
        }

        m_indexerPosition = m_indexer.getPosition();
        m_indexerVelocity = m_indexer.getVelocity();
        m_indexerAppliedVolts = m_indexer.getMotorVoltage();
        m_indexerStatorCurrent = m_indexer.getStatorCurrent();
        m_indexerSupplyCurrent = m_indexer.getSupplyCurrent();
        m_indexerTemp = m_indexer.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_indexerPosition,
            m_indexerVelocity,
            m_indexerAppliedVolts,
            m_indexerStatorCurrent,
            m_indexerSupplyCurrent
        );

        ParentDevice.optimizeBusUtilizationForAll(m_indexer);

        m_kickup = new SparkMax(kickupId, MotorType.kBrushless);

        SparkMaxConfig kickupConfig = new SparkMaxConfig();

        if (kSupplyCurrentLimitEnable)
            kickupConfig.smartCurrentLimit((int)kSupplyCurrentLimit);

        kickupConfig.encoder.positionConversionFactor(1 / kKickupGearRatio);
        kickupConfig.inverted(true);

        m_kickup.configure(kickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_kickupEncoder = m_kickup.getEncoder();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        StatusCode status = BaseStatusSignal.refreshAll(
            m_indexerPosition,
            m_indexerVelocity,
            m_indexerAppliedVolts,
            m_indexerStatorCurrent,
            m_indexerStatorCurrent,
            m_indexerTemp
        );

        inputs.indexerConnected = m_indexerConnectedDebounce.calculate(status.isOK());
        inputs.indexerPositionRad = m_indexerPosition.getValue().in(Units.Radians);
        inputs.indexerVelocityRadPerSec = m_indexerVelocity.getValue().in(Units.RadiansPerSecond);
        inputs.indexerAppliedVolts = m_indexerAppliedVolts.getValue().in(Units.Volts);
        inputs.indexerStatorCurrentAmps = m_indexerStatorCurrent.getValue().in(Units.Amps);
        inputs.indexerSupplyCurrentAmps = m_indexerSupplyCurrent.getValue().in(Units.Amp);
        inputs.indexerTempCelsius = m_indexerTemp.getValue().in(Units.Celsius);

        inputs.kickupPositionRad = Conversions.rotationsToRadians(m_kickupEncoder.getPosition());
        inputs.kickupVelocityRadPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(
            m_kickupEncoder.getVelocity()
        );
        inputs.kickupAppliedVolts = m_kickup.getAppliedOutput();
        inputs.kickupStatorCurrentAmps = m_kickup.getOutputCurrent();
        // P = I_stator * V_stator = I_supply * V_supply
        // I_supply = I_stator * V_stator / V_supply
        inputs.kickupSupplyCurrentAmps = inputs.kickupStatorCurrentAmps
            * inputs.kickupAppliedVolts
            / m_kickup.getBusVoltage();
        inputs.kickupTempCelsius = m_kickup.getMotorTemperature();
    }

    @Override
    public void runVoltage(double indexerVolts, double kickupVolts) {
        m_indexer.setControl(m_indexerVoltageOut.withOutput(indexerVolts));
        // m_kickup.setVoltage(kickupVolts);
    }
}
