
package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
    private final IntakeIOInputsPivot m_inputsPivot = new IntakeIOInputsPivot();
    private final IntakeIOInputsRoller m_inputsRoller = new IntakeIOInputsRoller();

    @Override
    public void updateInputsPivot(IntakeIOInputsPivot inputs) {
        inputs.connected = true;
        inputs.positionRad = m_inputsPivot.positionRad;
        inputs.velocityRadPerSec = m_inputsPivot.velocityRadPerSec;
        inputs.appliedVolts = m_inputsPivot.appliedVolts;
        inputs.statorCurrentAmps = m_inputsPivot.statorCurrentAmps;
        inputs.supplyCurrentAmps = m_inputsPivot.supplyCurrentAmps;
        inputs.tempCelsius = m_inputsPivot.tempCelsius;
    }

    @Override
    public void setVoltagePivot(double volts) {
        m_inputsPivot.appliedVolts = volts;
    }

    @Override
    public void setVelocityPivot(double velocityRadPerSec) {
        m_inputsPivot.velocityRadPerSec = velocityRadPerSec;
    }

    @Override
    public void updateInputsRoller(IntakeIOInputsRoller inputs) {
        inputs.connected = true;
        inputs.positionRad = m_inputsRoller.positionRad;
        inputs.velocityRadPerSec = m_inputsRoller.velocityRadPerSec;
        inputs.appliedVolts = m_inputsRoller.appliedVolts;
        inputs.statorCurrentAmps = m_inputsRoller.statorCurrentAmps;
        inputs.supplyCurrentAmps = m_inputsRoller.supplyCurrentAmps;
        inputs.tempCelsius = m_inputsRoller.tempCelsius;
    }

    @Override
    public void setVoltageRoller(double volts) {
        m_inputsRoller.appliedVolts = volts;
    }

    @Override
    public void setVelocityRoller(double velocityRadPerSec) {
        m_inputsRoller.velocityRadPerSec = velocityRadPerSec;
    }
}



