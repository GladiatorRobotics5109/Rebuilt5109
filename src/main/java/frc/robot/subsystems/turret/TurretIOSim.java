package frc.robot.subsystems.turret;

public class TurretIOSim implements TurretIO {
    private double m_desiredPosition;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = true;

        inputs.positionRad = m_desiredPosition;
    }

    @Override
    public void runPosition(double position) { m_desiredPosition = position; }
}
