package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    // The user's desired scrolling gradient pattern
    private final LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kDarkOrange);
    private static final Distance kLedSpacing = Meters.of(1 / 120.0);
    private final LEDPattern m_scrollingPattern = gradient.scrollAtAbsoluteSpeed(MetersPerSecond.of(.6), kLedSpacing);


    // Enum to represent the different states of the LEDs
    public enum LedState {
        SCROLLING_GRADIENT,
        BLUE,
        RED,
        DarkOrange
    }

    private LedState m_currentState;

    public LEDSubsystem() {
        // Initialize the LED strip and buffer from constants
        m_led = new AddressableLED(LedConstants.LED_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGTH);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the default state
        m_currentState = LedState.SCROLLING_GRADIENT;

        // Start the LED output
        m_led.start();
    }

    /**
     * Cycles to the next LED state.
     * SCROLLING_GRADIENT -> BLUE -> RED -> SCROLLING_GRADIENT
     */
    public void cycleState() {
        switch (m_currentState) {
            case SCROLLING_GRADIENT:
                m_currentState = LedState.BLUE;
                break;
            case BLUE:
                m_currentState = LedState.RED;
                break;
            case RED:
                m_currentState = LedState.SCROLLING_GRADIENT;
                break;
            case DarkOrange:
                m_currentState = LedState.SCROLLING_GRADIENT;
                break;
        }
    }

    @Override
    public void periodic() {
        // This method is called once per scheduler run and applies the current state to the buffer
        switch (m_currentState) {
            case SCROLLING_GRADIENT:
                setScrollingGradient();
                break;
            case BLUE:
                setSolidColor(Color.kBlue);
                break;
            case RED:
                setSolidColor(Color.kRed);
                break;
            case DarkOrange:
                setSolidColor(Color.kDarkOrange);
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
     * Applies the scrolling gradient pattern to the buffer.
     */
    private void setScrollingGradient() {
        m_scrollingPattern.applyTo(m_ledBuffer);
    }
}

