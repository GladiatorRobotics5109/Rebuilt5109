package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeIOTalonFX implements IntakeIO {
    private TalonFX m_rollers;
    private TalonFX m_pivot;

    public PositionVoltage m_pivotPositionVoltage = new PositionVoltage(0.0);
    private VoltageOut m_pivotVoltageOut = new VoltageOut(0.0);
    private VoltageOut m_rollersVoltageOut = new VoltageOut(0.0);

    private TalonFXConfiguration m_rollersConfig = new TalonFXConfiguration();
    private TalonFXConfiguration m_pivotConfig = new TalonFXConfiguration();

    private final Debouncer m_rollersConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer m_pivotConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    private final StatusSignal<Angle> m_rollersPosition;
    private final StatusSignal<AngularVelocity> m_rollersVelocity;
    private final StatusSignal<Voltage> m_rollersAppliedVolts;
    private final StatusSignal<Current> m_rollersStatorCurrent;
    private final StatusSignal<Current> m_rollersSupplyCurrent;
    private final StatusSignal<Temperature> m_rollersTemp;

    private final StatusSignal<Angle> m_pivotPosition;
    private final StatusSignal<AngularVelocity> m_pivotVelocity;
    private final StatusSignal<Voltage> m_pivotAppliedVolts;
    private final StatusSignal<Current> m_pivotStatorCurrent;
    private final StatusSignal<Current> m_pivotSupplyCurrent;
    private final StatusSignal<Temperature> m_pivotTemp;

    public IntakeIOTalonFX(int rollersId, int pivotId, CANBus canbus) {
        m_rollers = new TalonFX(rollersId, CANBus.roboRIO());
        m_pivot = new TalonFX(pivotId, CANBus.roboRIO());

        m_rollersConfig.CurrentLimits.StatorCurrentLimit = kRollersStatorCurrentLimit;
        m_rollersConfig.CurrentLimits.StatorCurrentLimitEnable = kRollersStatorCurrentLimitEnable;

        m_rollersConfig.CurrentLimits.SupplyCurrentLimit = kRollersSupplyCurrentLimit;
        m_rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = kRollersSupplyCurrentLimitEnable;

        m_rollersConfig.MotorOutput.Inverted = kRollersInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        m_rollersConfig.MotorOutput.NeutralMode = kRollersBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        m_rollersConfig.Feedback.SensorToMechanismRatio = kRollersGearRatio;

        m_rollers.getConfigurator().apply(m_rollersConfig);

        m_pivotConfig.CurrentLimits.StatorCurrentLimit = kPivotStatorCurrentLimit;
        m_pivotConfig.CurrentLimits.StatorCurrentLimitEnable = kPivotStatorCurrentLimitEnable;

        m_pivotConfig.CurrentLimits.SupplyCurrentLimit = kPivotSupplyCurrentLimit;
        m_pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = kPivotSupplyCurrentLimitEnable;

        m_pivotConfig.MotorOutput.Inverted = kPivotInvert
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        m_pivotConfig.MotorOutput.NeutralMode = kPivotBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        m_pivotConfig.Feedback.SensorToMechanismRatio = kPivotGearRatio;

        m_pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        m_pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kPivotMaxPosition.getRotations()
            * kPivotGearRatio;

        m_pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kPivotMinPosition.getRotations()
            * kPivotGearRatio;
        m_pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        m_pivotConfig.Slot0.kP = kPivotP;
        m_pivotConfig.Slot0.kI = kPivotI;
        m_pivotConfig.Slot0.kD = kPivotD;
        m_pivotConfig.Slot0.kS = kPivotS;
        m_pivotConfig.Slot0.kV = kPivotV;

        m_pivot.getConfigurator().apply(m_pivotConfig);

        m_rollersPosition = m_rollers.getPosition();
        m_rollersVelocity = m_rollers.getVelocity();
        m_rollersAppliedVolts = m_rollers.getMotorVoltage();
        m_rollersStatorCurrent = m_rollers.getStatorCurrent();
        m_rollersSupplyCurrent = m_rollers.getSupplyCurrent();
        m_rollersTemp = m_rollers.getDeviceTemp();

        m_pivotPosition = m_pivot.getPosition();
        m_pivotVelocity = m_pivot.getVelocity();
        m_pivotAppliedVolts = m_pivot.getMotorVoltage();
        m_pivotStatorCurrent = m_pivot.getStatorCurrent();
        m_pivotSupplyCurrent = m_pivot.getSupplyCurrent();
        m_pivotTemp = m_pivot.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_rollersPosition,
            m_rollersVelocity,
            m_rollersAppliedVolts,
            m_rollersStatorCurrent,
            m_rollersSupplyCurrent,
            m_rollersTemp,
            m_pivotPosition,
            m_pivotVelocity,
            m_pivotAppliedVolts,
            m_pivotStatorCurrent,
            m_pivotSupplyCurrent,
            m_pivotTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(m_rollers, m_pivot);

        m_pivot.setPosition(kPivotStowedPosition.getRotations());
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        StatusCode rollersStatus = BaseStatusSignal.refreshAll(
            m_rollersPosition,
            m_rollersVelocity,
            m_rollersAppliedVolts,
            m_rollersStatorCurrent,
            m_rollersSupplyCurrent,
            m_rollersTemp
        );

        inputs.rollersConnected = m_rollersConnectedDebounce.calculate(rollersStatus.isOK());
        inputs.rollersPositionRot = m_rollersPosition.getValue().in(Units.Rotations);
        inputs.rollersVelocityRPM = m_rollersVelocity.getValue().in(Units.RPM);
        inputs.rollersAppliedVolts = m_rollersAppliedVolts.getValue().in(Units.Volts);
        inputs.rollersStatorCurrentAmps = m_rollersStatorCurrent.getValue().in(Units.Amps);
        inputs.rollersSupplyCurrentAmps = m_rollersSupplyCurrent.getValue().in(Units.Amps);
        inputs.rollersTempCelsius = m_rollersTemp.getValue().in(Units.Celsius);

        StatusCode pivotStatus = BaseStatusSignal.refreshAll(
            m_pivotPosition,
            m_pivotVelocity,
            m_pivotAppliedVolts,
            m_pivotStatorCurrent,
            m_pivotSupplyCurrent,
            m_pivotTemp
        );

        inputs.pivotConnected = m_pivotConnectedDebounce.calculate(pivotStatus.isOK());
        inputs.pivotPositionRot = m_pivotPosition.getValue().in(Units.Rotations);
        inputs.pivotVelocityRPM = m_pivotVelocity.getValue().in(Units.RPM);
        inputs.pivotAppliedVolts = m_pivotAppliedVolts.getValue().in(Units.Volts);
        inputs.pivotStatorCurrentAmps = m_pivotStatorCurrent.getValue().in(Units.Amps);
        inputs.pivotSupplyCurrentAmps = m_pivotSupplyCurrent.getValue().in(Units.Amps);
        inputs.pivotTempCelsius = m_pivotTemp.getValue().in(Units.Celsius);
    }

    @Override
    public void runPivotVoltage(double volts) {
        m_pivot.setControl(m_pivotVoltageOut.withOutput(volts));
    }

    @Override
    public void runPivotPosition(Rotation2d position) {
        m_pivot.setControl(m_pivotPositionVoltage.withPosition(position.getRotations()));
    }

    @Override
    public void runRollersVoltage(double volts) {
        m_rollers.setControl(m_rollersVoltageOut.withOutput(volts));
    }
}
