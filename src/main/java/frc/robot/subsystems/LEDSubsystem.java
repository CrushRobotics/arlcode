package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    // Enum to represent the different states of the LEDs
    public enum LedState {
        RAINBOW,
        BLUE,
        RED,
        ORANGE
    }

    private LedState m_currentState;

    public LEDSubsystem() {
        // Initialize the LED strip and buffer from constants
        m_led = new AddressableLED(LedConstants.LED_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the default state
        m_currentState = LedState.RAINBOW;

        // Start the LED output
        m_led.start();
    }

    /**
     * Cycles to the next LED state.
     * RAINBOW -> BLUE -> RED -> RAINBOW
     */
    public void cycleState() {
        switch (m_currentState) {
            case RAINBOW:
                m_currentState = LedState.BLUE;
                break;
            case BLUE:
                m_currentState = LedState.RED;
                break;
            case RED:
                m_currentState = LedState.RAINBOW;
                break;
            case ORANGE:
                m_currentState = LedState.ORANGE;
                break;
        }
    }

    @Override
    public void periodic() {
        // This method is called once per scheduler run and applies the current state to the buffer
        switch (m_currentState) {
            case RAINBOW:
                setRainbow();
                break;
            case BLUE:
                setSolidColor(Color.kBlue);
                break;
            case RED:
                setSolidColor(Color.kRed);
                break;
            case ORANGE:
                setSolidColor(Color.kOrange);
                break;
        }
        // Write the buffer to the LED strip
        m_led.setData(m_ledBuffer);
    }

    /**
     * Fills the buffer with a solid color.
     * @param color The color to set.
     */
    private void setSolidColor(Color color) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, color);
        }
    }

    /**
     * Fills the buffer with a rainbow pattern.
     */
    private void setRainbow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (int) ((System.currentTimeMillis() / 10) + (i * 180.0 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
    }
}

