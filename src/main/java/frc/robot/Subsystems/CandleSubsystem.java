package frc.robot.Subsystems;


import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CandleConstants;

public class CandleSubsystem extends SubsystemBase {
    private static final CANdle candle = new CANdle(CandleConstants.kCandleId, "ShooterCAN");

    public CandleSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.RGB; // Change to GRB if colors are swapped
        config.LED.BrightnessScalar = 0.5;
        candle.getConfigurator().apply(config);
    }

    public static void setBlinkGreen() {
        // In Phoenix 6 (2025+), the constructor only takes (numLed, startIdx).
        // Use .withColor and .withSpeed to configure the animation.
        StrobeAnimation strobe = new StrobeAnimation(CandleConstants.kLedCount, 0)
            .withColor(new RGBWColor(1,83,47));
            candle.setControl(strobe);
    }

    public static void setBlinkRed() {
        // In Phoenix 6 (2025+), the constructor only takes (numLed, startIdx).
        // Use .withColor and .withSpeed to configure the animation.
        StrobeAnimation strobe = new StrobeAnimation(CandleConstants.kLedCount, 0)
            .withColor(new RGBWColor(255,0,0));
        candle.setControl(strobe);
    }

    public static void setBlinkYellow() {
        // In Phoenix 6 (2025+), the constructor only takes (numLed, startIdx).
        // Use .withColor and .withSpeed to configure the animation.
        StrobeAnimation strobe = new StrobeAnimation(CandleConstants.kLedCount, 0)
            .withColor(new RGBWColor(255,255,0));
        candle.setControl(strobe);
    }

    public static void setBlinkBlue() {
        // In Phoenix 6 (2025+), the constructor only takes (numLed, startIdx).
        // Use .withColor and .withSpeed to configure the animation.
        StrobeAnimation strobe = new StrobeAnimation(CandleConstants.kLedCount, 0)
            .withColor(new RGBWColor(0,0,255));
        candle.setControl(strobe);
    }

    public static void setRainbow() {
        RainbowAnimation rainbow = new RainbowAnimation(CandleConstants.kLedCount, 0)
            .withBrightness(0.5)
            .withFrameRate(0.5);
        candle.setControl(rainbow);
    }

    public static void stop() {
        // To stop a persistent animation, apply a "black" strobe or static color
        StrobeAnimation off = new StrobeAnimation(CandleConstants.kLedCount, 0)
            .withColor(new RGBWColor(0, 0, 0, 0));
        candle.setControl(off);
    }
}
