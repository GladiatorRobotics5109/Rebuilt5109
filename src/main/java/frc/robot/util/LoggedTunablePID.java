package frc.robot.util;

public class LoggedTunablePID {
    private final String m_key;

    private final LoggedTunableNumber m_p;
    private final LoggedTunableNumber m_i;
    private final LoggedTunableNumber m_d;

    public LoggedTunablePID(String key) {
        m_key = key;

        m_p = new LoggedTunableNumber(key + "/p");
        m_i = new LoggedTunableNumber(key + "/i");
        m_d = new LoggedTunableNumber(key + "/d");
    }

    public LoggedTunablePID(String key, double p, double i, double d) {
        m_key = key;

        m_p = new LoggedTunableNumber(key + "/p", p);
        m_i = new LoggedTunableNumber(key + "/i", i);
        m_d = new LoggedTunableNumber(key + "/d", d);
    }

    public boolean hasChanged(int id) {
        return m_p.hasChanged(id) || m_i.hasChanged(id) || m_d.hasChanged(id);
    }

    public double getP() { return m_p.get(); }

    public double getI() { return m_i.get(); }

    public double getD() { return m_d.get(); }
}
