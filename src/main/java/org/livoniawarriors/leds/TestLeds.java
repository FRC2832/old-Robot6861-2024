package org.livoniawarriors.leds;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class TestLeds extends Command {
    private ILedSubsystem leds;
    private AddressableLEDBuffer ledBuffer;

    private IntegerSubscriber subIndex;
    private IntegerSubscriber subHue;
    private IntegerSubscriber subSat;
    private IntegerSubscriber subValue;

    public TestLeds(ILedSubsystem leds) {
        this.leds = leds;
        addRequirements(leds);
        ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        // normally this should be in the constructor, but moving to init so it won't
        // always be in NT
        subIndex = UtilFunctions.getNtSub("/TestLeds/Index", -1);
        subHue = UtilFunctions.getNtSub("/TestLeds/Hue", 0);
        subSat = UtilFunctions.getNtSub("/TestLeds/Saturation", 20);
        subValue = UtilFunctions.getNtSub("/TestLeds/Value", 0);
    }

    @Override
    public void execute() {
        // get color
        Color newColor = Color.fromHSV((int) subHue.get(), (int) subSat.get(), (int) subValue.get());
        int pos = (int) subValue.get();

        // check if we are doing individual index
        if (pos >= 0) {
            // fill the whole string with black
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, Color.kBlack);
            }
            // color the specific LED
            int max = leds.getLength() - 1;
            if (pos > max)
                pos = max;
            ledBuffer.setLED(pos, newColor);
        } else {
            // fill the whole string with the color
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, newColor);
            }
        }

        leds.setData(ledBuffer);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Do nothing
    }
}
