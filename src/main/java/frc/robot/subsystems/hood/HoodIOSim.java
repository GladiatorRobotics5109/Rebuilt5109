package frc.robot.subsystems.hood;

import static frc.robot.Constants.HoodConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import edu.wpi.first.math.MathUtil;

public class HoodIOSim implements HoodIO {

    private final DCMotorSim m_motor;

    private final PIDController m_pid = new PIDController(kP, kI, kD);

    private double m_desiredPositionRad;
    private boolean m_hasDesiredPosition;

    // TO-DO: CHANGE KRAKEN MODEL (i forgot the number)

    public HoodIOSim() {
        m_motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), kSimMOI, 1),
            DCMotor.getKrakenX44(1)
        );
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        m_motor.update(Robot.defaultPeriodSecs);

        inputs.connected = true;
        inputs.positionRad = m_motor.getAngularPositionRad();
        inputs.velocityRadPerSec = m_motor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = m_motor.getInputVoltage();
        inputs.statorCurrentAmps = Math.abs(m_motor.getCurrentDrawAmps());

        if (m_hasDesiredPosition) {
            double volts = m_pid.calculate(inputs.velocityRadPerSec, m_desiredPositionRad);
            m_motor.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
        }
    }

    @Override
    public void setPosition(double positionRad) {
        m_desiredPositionRad = positionRad;
        m_hasDesiredPosition = true;
    }

    @Override
    public void setPID(double p, double i, double d) {
        m_pid.setPID(p, i, d);
        m_pid.setTolerance(kToleranceRad); 
    }
}
