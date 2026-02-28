package frc.robot.util;

public class LoggedTunableFF {
    private final String m_key;

    private final LoggedTunableNumber m_s;
    private final LoggedTunableNumber m_v;
    private final LoggedTunableNumber m_a;
    private final LoggedTunableNumber m_g;

    public LoggedTunableFF(String key) {
        m_key = key;

        m_s = new LoggedTunableNumber(key + "/kS");
        m_v = new LoggedTunableNumber(key + "/kV");
        m_a = new LoggedTunableNumber(key + "/kA");
        m_g = new LoggedTunableNumber(key + "/kG");
    }

    public LoggedTunableFF(String key, double s, double v, double a, double g) {
        m_key = key;

        m_s = new LoggedTunableNumber(key + "/kS", s);
        m_v = new LoggedTunableNumber(key + "/kV", v);
        m_a = new LoggedTunableNumber(key + "/kA", a);
        m_g = new LoggedTunableNumber(key + "/kG", g);
    }

    public boolean hasChanged(int id) {
        return m_s.hasChanged(id) || m_v.hasChanged(id) || m_a.hasChanged(id) || m_g.hasChanged(id);
    }

    public double getS() { return m_s.getAsDouble(); }

    public double getV() { return m_v.getAsDouble(); }

    public double getA() { return m_a.getAsDouble(); }

    public double getG() { return m_g.getAsDouble(); }
}
