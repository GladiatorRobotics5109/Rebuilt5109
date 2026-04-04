package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {
    private double m_desiredPosition;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = true;

        inputs.positionRad = MathUtil.clamp(
            m_desiredPosition,
            TurretConstants.kMinPosition.getRadians(),
            TurretConstants.kMaxPosition.getRadians()
        );
    }

    @Override
    public void runPosition(double position) { m_desiredPosition = position; }
}
