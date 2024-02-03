package org.livoniawarriors.lightdriveleds;

public class Status {
    private byte m_raw;
    private Mode m_mode;

    public Status() {
        this.m_raw = 0;
        this.m_mode = Mode.NONE;
    }

    public byte GetTripped() {
        return (byte) ((this.m_raw & 0xF0) >> 4);
    }

    public Boolean IsEnabled() {
        if ((this.m_raw & 0x1) > 0) {
            return true;
        }
        return false;
    }

    public Mode GetMode() {
        return this.m_mode;
    }

    public Byte GetRaw() {
        return this.m_raw;
    }

    public void SetRaw(final byte raw) {
        this.m_raw = raw;
    }

    public enum Mode {
        NONE("NONE", 0),
        IDLE("IDLE", 1),
        PWM("PWM", 2),
        CAN("CAN", 3),
        SERIAL("SERIAL", 4);

        private Mode(final String s, final int n) {
        }
    }
}