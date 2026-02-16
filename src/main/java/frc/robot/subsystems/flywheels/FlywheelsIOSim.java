package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

import static frc.robot.Constants.FlywheelsConstants.*;

public class FlywheelsIOSim implements FlywheelsIO {
    private final DCMotorSim m_motor;

    public FlywheelsIOSim() {
        m_motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), kSimMOI, 1),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        m_motor.update(Robot.defaultPeriodSecs);

        inputs.connected = true;
        inputs.positionRot = m_motor.getAngularPositionRotations();
        inputs.velocityRPM = m_motor.getAngularVelocityRPM();
        inputs.appliedVolts = m_motor.getInputVoltage();
        inputs.statorCurrentAmps = Math.abs(m_motor.getCurrentDrawAmps());

    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    public void simulateShot() {
        m_motor.setAngularVelocity(
            Math.min(m_motor.getAngularVelocityRadPerSec() - Units.rotationsPerMinuteToRadiansPerSecond(237), 0)
        );
    }
}
