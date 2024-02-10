package org.livoniawarriors.lightdriveleds;

import java.nio.*;
import edu.wpi.first.wpilibj.util.*;
import edu.wpi.first.hal.can.*;
import edu.wpi.first.hal.util.UncleanStatusException;

public final class LightDriveCAN {
    private static int ldAddr;
    private ByteBuffer matrix;
    private RxPacket rx;
    // private boolean m_init;
    private byte[] rxData;
    private static ByteBuffer timestamp;
    private static ByteBuffer rxId;

    static {
        LightDriveCAN.ldAddr = 33882112;
    }

    public LightDriveCAN() {
        this.matrix = ByteBuffer.allocate(16);
        // this.m_init = false;
        this.rx = new RxPacket();
        (LightDriveCAN.timestamp = ByteBuffer.allocateDirect(4)).order(ByteOrder.LITTLE_ENDIAN);
        (LightDriveCAN.rxId = ByteBuffer.allocateDirect(4)).order(ByteOrder.LITTLE_ENDIAN);
    }

    public LightDriveCAN(final int addr) {
    }

    public void update() {
        final byte[] txData = new byte[8];
        LightDriveCAN.rxId.putInt(LightDriveCAN.ldAddr + 4);
        LightDriveCAN.rxId.rewind();
        try {
            this.matrix.get(txData, 0, 8);
            CANJNI.FRCNetCommCANSessionMuxSendMessage(LightDriveCAN.ldAddr, txData, 100);
            this.matrix.get(txData, 0, 8);
            CANJNI.FRCNetCommCANSessionMuxSendMessage(LightDriveCAN.ldAddr + 1, txData, 100);
        } catch (UncleanStatusException ex) {
        }
        this.matrix.rewind();
        try {
            this.rxData = CANJNI.FRCNetCommCANSessionMuxReceiveMessage(LightDriveCAN.rxId.asIntBuffer(), 536870911,
                    LightDriveCAN.timestamp);
            if (this.rxData.length > 7) {
                this.rx.setBytes(this.rxData);
            }
        } catch (CANMessageNotFoundException ex2) {
        }
    }

    public float getCurrent(final int ch) {
        float current = 0.0f;
        switch (ch) {
            case 1: {
                current = this.rx.i1;
                break;
            }
            case 2: {
                current = this.rx.i2;
                break;
            }
            case 3: {
                current = this.rx.i3;
                break;
            }
            case 4: {
                current = this.rx.i4;
                break;
            }
            default: {
                current = -10.0f;
                break;
            }
        }
        return current / 10.0f;
    }

    public float getTotalCurrent() {
        return (this.rx.i1 + this.rx.i2 + this.rx.i3 + this.rx.i4) / 10.0f;
    }

    public float getVoltage() {
        return this.rx.vin / 10.0f;
    }

    public int getFWVersion() {
        return this.rx.fw;
    }

    public Status getStatus() {
        return this.rx.status;
    }

    public int getPWMs(final int ch) {
        if (ch > 2 || ch < 1) {
            return -1;
        }
        return (ch > 1) ? (this.rx.pwmVals >> 8) : (this.rx.pwmVals & 0xFF);
    }

    public void setColor(int ch, final Color color) {
        if (ch < 1 || ch > 4) {
            return;
        }
        ch = --ch * 3;
        this.matrix.array()[ch] = (byte) color.green;
        this.matrix.array()[ch + 1] = (byte) color.red;
        this.matrix.array()[ch + 2] = (byte) color.blue;
    }

    public void setColor(int ch, final Color color, final double brightness) {
        if (ch < 1 || ch > 4) {
            return;
        }
        byte red = (byte) (color.red * brightness);
        byte green = (byte) (color.green * brightness);
        byte blue = (byte) (color.blue * brightness);
        ch = --ch * 3;
        this.matrix.array()[ch] = green;
        this.matrix.array()[ch + 1] = red;
        this.matrix.array()[ch + 2] = blue;
    }

    public void setLevel(final int ch, final byte level) {
        if (ch < 1 || ch > 12 || level < 0 || level > 255) {
            return;
        }
        this.matrix.array()[ch] = level;
    }
}