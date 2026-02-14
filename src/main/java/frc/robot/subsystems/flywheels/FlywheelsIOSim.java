package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

import static frc.robot.Constants.FlywheelsConstants.*;

public class FlywheelsIOSim implements FlywheelsIO {
    private final DCMotorSim m_motor;

    private final BangBangController m_bang = new BangBangController(kBangBangTolerance);
    private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(kS, kV, kA);
    private double m_desiredVelocity;
    private boolean m_hasDesiredVelocity;
    private double m_appliedVolts;

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
        inputs.positionRad = m_motor.getAngularPositionRad();
        inputs.velocityRadPerSec = m_motor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = m_motor.getInputVoltage();
        inputs.statorCurrentAmps = Math.abs(m_motor.getCurrentDrawAmps());

        if (m_hasDesiredVelocity) {
            double volts = 12 * m_bang.calculate(inputs.velocityRadPerSec, m_desiredVelocity)
                + m_ff.calculate(m_desiredVelocity);
            m_motor.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
        }
    }

    @Override
    public void setVoltage(double volts) {
        m_hasDesiredVelocity = false;
        m_motor.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        m_desiredVelocity = velocityRadPerSec;
        m_hasDesiredVelocity = true;
    }

    @Override
    public void setPID(double p, double i, double d) {
        // m_pid.setPID(p, i, d);
    }

    public void simulateShot() {
        m_motor.setAngularVelocity(
            Math.min(m_motor.getAngularVelocityRadPerSec() - Units.rotationsPerMinuteToRadiansPerSecond(237), 0)
        );
    }
}
