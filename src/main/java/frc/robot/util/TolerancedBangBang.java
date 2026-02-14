package frc.robot.util;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;

@AllArgsConstructor
public class TolerancedBangBang {
    @Getter
    @Setter
    private double m_tolerance;

    public double calculate(double measurement, double setpoint) {
        return setpoint - measurement > m_tolerance ? 1 : 0;
    }
}
