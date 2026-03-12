package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

import static frc.robot.Constants.IndexerConstants.*;

public class IndexerIOSim implements IndexerIO {

    private final DCMotorSim m_motor;
    private double m_appliedVolts = 0.0;

    public IndexerIOSim() {

        // Create physics model of 1 Kraken motor
        m_motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                kSimMOI,
                kIndexerGearRatio
            ),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {

        // Advance physics by 20ms
        m_motor.update(Robot.defaultPeriodSecs);

        inputs.indexerConnected = true;

        inputs.indexerPositionRad = m_motor.getAngularPositionRad();
        inputs.indexerVelocityRadPerSec = m_motor.getAngularVelocityRadPerSec();
        inputs.indexerAppliedVolts = m_appliedVolts;

        inputs.indexerStatorCurrentAmps = Math.abs(m_motor.getCurrentDrawAmps());

        inputs.indexerSupplyCurrentAmps = Math.abs(m_motor.getCurrentDrawAmps());

        inputs.indexerTempCelsius = 30.0;
    }

    @Override
    public void runVoltage(double indexerVolts, double kickupVolts) {

        m_appliedVolts = MathUtil.clamp(indexerVolts, -12.0, 12.0);
        m_motor.setInputVoltage(m_appliedVolts);
    }
}
