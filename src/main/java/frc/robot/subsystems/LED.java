package frc.robot.subsystems;

import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;
import frc.robot.utils.FieldUtils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LED extends SubsystemBase{

	public static int m_blinkSpeed;
	public static int m_checkeredSpeed;
	public static int m_checkeredBlinkSpeed;
	public static int m_slideSpeed;
	public static int m_gradientSpeed;
	public static int m_rainbowSpeed;
    public static class LEDPattern {


        public enum PatternType {
            SOLID,
            BLINK,
            CHECKERED,
            CHECKERED_BLINK,
            SLIDE,
            GRADIENT,
            RAINBOW,
            RUNNABLE
        }
        private PatternType m_patternType;
        private Color m_foreground;
        private Color m_background;
        private int m_speed;


        public LEDPattern(PatternType patternType, Color foreground, Color background, int speed) {
            m_patternType = patternType;
            m_foreground = foreground;
            m_background = background;
            m_speed = speed;
        }

		/**
		 * Reassign the pattern type, the expression for the pattern.
		 * @param patternType New pattern type.
		 */
		public void setPatternType(PatternType patternType) {
			m_patternType = patternType;
		}

		/**
		 * Reassign foreground color, the 'primary' color of the pattern.
		 * @param foreground New foreground color.
		 */
		public void setForeground(Color foreground) {
			m_foreground = foreground;
		}

		/**
		 * Reassign background color, the 'secondary' color of the pattern.
		 * @param background New background color.
		 */
		public void setBackground(Color background) {
			m_background = background;
		}

		/**
		 * Reassign speed, typically a multiplier against time.
		 * <i>NOTE: Using setSpeed() will result in the pattern shifting its position, because
		 * 'position' isn't actually held state, it's only a product of time x speed.</i>
		 * @param speed New speed for the pattern.
		 */
		public void setSpeed(int speed) {
			m_speed = speed;
		}

        /**
         * Calculates a color from the LEDPattern's pattern type, stored colors, and speed.
         * @param lightIndex The index of the LED whose color is being calculated.
         * @param tick The time in MS. Should be the same between every call at one time.
		 * @param numLights The number of lights in the string of LEDs.
         * @return A color to assign to the LED.
         */
        public Color getColor(int lightIndex, int tick, int numLights) {
            switch (m_patternType) {
                case SOLID:
                    return m_foreground;
                case BLINK: 
                    return (((tick*m_speed)/1000)%2 == 0) ? m_foreground : m_background;
                case CHECKERED:
                    return (lightIndex%2 == 0) ? m_foreground : m_background;
                case CHECKERED_BLINK:
                    return (((tick*m_speed)/1000+lightIndex)%2 == 0) ? m_foreground : m_background;
                case SLIDE:
                    return (((tick*m_speed)/1000+lightIndex)%4 == 0) ? m_foreground : m_background;
                case GRADIENT:
                    double dt = (tick*m_speed)/1000.0+(double)lightIndex/numLights;
                    return Color.lerpRGB(m_foreground, m_background, (-Math.cos(Math.PI*2*dt)+1)/2);
                case RAINBOW:
                    return Color.fromHSV(((tick*m_speed)/1000)%180, 255, 255);
                case RUNNABLE:
                    return runnable(lightIndex, tick, numLights);
            }
            return m_foreground;
        }

        /**
		 * Overridable method; extend LEDPattern and override runnable() to program
		 * the pattern. Made as a separate method so getColor doesn't have to be
		 * overriden, so the built-in patterns can be kept.
         * @param lightIndex The index of the LED whose color is being calculated.
         * @param tick The time in MS. Should be the same between every call at one time.
		 * @param numLights The number of lights in the string of LEDs.
         * @return A color to assign to the LED.
		 */
        public Color runnable(int lightIndex, int tick, int numLights) {return Color.kWhite;}

		// LEDPattern Constants
		// Solids
		public static final LEDPattern kSolidWhite =
			new LEDPattern(PatternType.SOLID, Color.kWhite, Color.kWhite, 1);
		public static final LEDPattern kSolidMaroon =
			new LEDPattern(PatternType.SOLID, Color.kRed, Color.kMaroon, 1);
		public static final LEDPattern kSolidRed =
			new LEDPattern(PatternType.SOLID, Color.kRed, Color.kRed, 1);
		public static final LEDPattern kSolidOrange =
			new LEDPattern(PatternType.SOLID, Color.kOrange, Color.kOrange, 1);
		public static final LEDPattern kSolidYellow =
			new LEDPattern(PatternType.SOLID, Color.kYellow, Color.kYellow, 1);
		public static final LEDPattern kSolidGreen =
			new LEDPattern(PatternType.SOLID, Color.kGreen, Color.kGreen, 1);
		public static final LEDPattern kSolidLightBlue =
			new LEDPattern(PatternType.SOLID, Color.kLightBlue, Color.kLightBlue, 1);
		public static final LEDPattern kSolidBlue =
			new LEDPattern(PatternType.SOLID, Color.kBlue, Color.kBlue, 1);
		public static final LEDPattern kSolidPurple =
			new LEDPattern(PatternType.SOLID, Color.kPurple, Color.kPurple, 1);
		public static final LEDPattern kSolidPink =
			new LEDPattern(PatternType.SOLID, Color.kPurple, Color.kPink, 1);
		public static final LEDPattern kSolidLightPink =
			new LEDPattern(PatternType.SOLID, Color.kLightPink, Color.kLightPink, 1);
		public static final LEDPattern kSolidBlack =
			new LEDPattern(PatternType.SOLID, Color.kBlack, Color.kBlack, 1);

		// Blink
		
		public static final LEDPattern kBlinkWhite =
			new LEDPattern(PatternType.BLINK, Color.kWhite, Color.kBlack, m_blinkSpeed);
			public static final LEDPattern kBlinkMaroon =
			new LEDPattern(PatternType.BLINK, Color.kMaroon, Color.kBlack, m_blinkSpeed);
		public static final LEDPattern kBlinkRed =
			new LEDPattern(PatternType.BLINK, Color.kRed, Color.kBlack, m_blinkSpeed);
		public static final LEDPattern kBlinkOrange =
			new LEDPattern(PatternType.BLINK, Color.kOrange, Color.kBlack, m_blinkSpeed);
		public static final LEDPattern kBlinkYellow =
			new LEDPattern(PatternType.BLINK, Color.kYellow, Color.kBlack, m_blinkSpeed);
		public static final LEDPattern kBlinkGreen =
			new LEDPattern(PatternType.BLINK, Color.kGreen, Color.kBlack, m_blinkSpeed);
		public static final LEDPattern kBlinkBlue =
			new LEDPattern(PatternType.BLINK, Color.kBlue, Color.kBlack, m_blinkSpeed);
		public static final LEDPattern kBlinkPurple =
			new LEDPattern(PatternType.BLINK, Color.kPurple, Color.kBlack, m_blinkSpeed);
		public static final LEDPattern kBlinkTeamColors =
			new LEDPattern(PatternType.BLINK, Color.kMaroon, Color.kGold, m_blinkSpeed);
		public static final LEDPattern kBlinkTeamColorsInverted =
			new LEDPattern(PatternType.BLINK, Color.kGold, Color.kMaroon, m_blinkSpeed);	

		// Checkered
		public static final LEDPattern kCheckeredWhite =
			new LEDPattern(PatternType.CHECKERED, Color.kWhite, Color.kBlack, m_checkeredSpeed);
		public static final LEDPattern kCheckeredMaroon =
			new LEDPattern(PatternType.CHECKERED, Color.kMaroon, Color.kBlack, m_checkeredSpeed);
		public static final LEDPattern kCheckeredRed =
			new LEDPattern(PatternType.CHECKERED, Color.kRed, Color.kBlack, m_checkeredSpeed);
		public static final LEDPattern kCheckeredOrange =
			new LEDPattern(PatternType.CHECKERED, Color.kOrange, Color.kBlack, m_checkeredSpeed);
		public static final LEDPattern kCheckeredYellow =
			new LEDPattern(PatternType.CHECKERED, Color.kYellow, Color.kBlack, m_checkeredSpeed);
		public static final LEDPattern kCheckeredGreen =
			new LEDPattern(PatternType.CHECKERED, Color.kGreen, Color.kBlack, m_checkeredSpeed);
		public static final LEDPattern kCheckeredBlue =
			new LEDPattern(PatternType.CHECKERED, Color.kBlue, Color.kBlack, m_checkeredSpeed);
		public static final LEDPattern kCheckeredPurple =
			new LEDPattern(PatternType.CHECKERED, Color.kPurple, Color.kBlack, m_checkeredSpeed);

		// Checkered
		public static final LEDPattern kCheckeredBlinkWhite =
			new LEDPattern(PatternType.CHECKERED_BLINK, Color.kWhite, Color.kBlack, m_checkeredBlinkSpeed);
		public static final LEDPattern kCheckeredBlinkMaroon =
			new LEDPattern(PatternType.CHECKERED_BLINK, Color.kMaroon, Color.kBlack, m_checkeredBlinkSpeed);
		public static final LEDPattern kCheckeredBlinkRed =
			new LEDPattern(PatternType.CHECKERED_BLINK, Color.kRed, Color.kBlack, m_checkeredBlinkSpeed);
		public static final LEDPattern kCheckeredBlinkOrange =
			new LEDPattern(PatternType.CHECKERED_BLINK, Color.kOrange, Color.kBlack, m_checkeredBlinkSpeed);
		public static final LEDPattern kCheckeredBlinkYellow =
			new LEDPattern(PatternType.CHECKERED_BLINK, Color.kYellow, Color.kBlack, m_checkeredBlinkSpeed);
		public static final LEDPattern kCheckeredBlinkGreen =
			new LEDPattern(PatternType.CHECKERED_BLINK, Color.kGreen, Color.kBlack, m_checkeredBlinkSpeed);
		public static final LEDPattern kCheckeredBlinkBlue =
			new LEDPattern(PatternType.CHECKERED_BLINK, Color.kBlue, Color.kBlack, m_checkeredBlinkSpeed);
		public static final LEDPattern kCheckeredBlinkPurple =
			new LEDPattern(PatternType.CHECKERED_BLINK, Color.kPurple, Color.kBlack, m_checkeredBlinkSpeed);

		// Slide
		public static final LEDPattern kSlideWhite =
			new LEDPattern(PatternType.SLIDE, Color.kWhite, Color.kBlack, m_slideSpeed);
		public static final LEDPattern kSlideRed =
			new LEDPattern(PatternType.SLIDE, Color.kRed, Color.kBlack, m_slideSpeed);
		public static final LEDPattern kSlideMaroon =
			new LEDPattern(PatternType.SLIDE, Color.kMaroon, Color.kBlack, m_slideSpeed);
		public static final LEDPattern kSlideOrange =
			new LEDPattern(PatternType.SLIDE, Color.kOrange, Color.kBlack, m_slideSpeed);
		public static final LEDPattern kSlideYellow =
			new LEDPattern(PatternType.SLIDE, Color.kYellow, Color.kBlack, m_slideSpeed);
		public static final LEDPattern kSlideGreen =
			new LEDPattern(PatternType.SLIDE, Color.kGreen, Color.kBlack, m_slideSpeed);
		public static final LEDPattern kSlideBlue =
			new LEDPattern(PatternType.SLIDE, Color.kBlue, Color.kBlack, m_slideSpeed);
		public static final LEDPattern kSlidePurple =
			new LEDPattern(PatternType.SLIDE, Color.kPurple, Color.kBlack, m_slideSpeed);
		public static final LEDPattern kSlideInverted =
			new LEDPattern(PatternType.SLIDE, Color.kBlack, Color.kWhite, m_slideSpeed);
		public static final LEDPattern kSlideTrans =
		new LEDPattern(PatternType.SLIDE, Color.kLightBlue, Color.kLightPink, m_gradientSpeed);

		// Gradient
		public static final LEDPattern kGradientBW =
			new LEDPattern(PatternType.GRADIENT, Color.kBlack, Color.kWhite, m_gradientSpeed);
			public static final LEDPattern kGradientTrans =
			new LEDPattern(PatternType.GRADIENT, Color.kLightBlue, Color.kLightPink, m_gradientSpeed);

		// Rainbow
		public static final LEDPattern kRainbow =
			new LEDPattern(PatternType.RAINBOW, Color.kWhite, Color.kWhite, 1);
    }

	public static class LEDSection {
		public enum Priority {
			CRITICAL(1),
			WARNING(2),
			INFO(3),
			BASIC(4);

			Priority(int power) {
				this.power = power;
			}

			private int power;

			public boolean greater(Priority other) {
				return this.power > other.power;
			}
		}

		private Priority m_priority;
		private LEDPattern m_pattern;

		public LEDSection(Priority priority, LEDPattern pattern) {
			m_priority = priority;
			m_pattern = pattern;
		}

		public Priority getPriority() {
			return m_priority;
		}

		public LEDPattern getPattern() {
			return m_pattern;
		}
	}

    private static LED m_LEDLights = null;

    private AddressableLED m_LED;
    private AddressableLEDBuffer m_LEDBuffer;

    private LEDSection[] m_sections;

	private LEDPattern m_fallbackPattern;

	private Mechanism2d m_stageMechanism;

    private LED() {
        super("LED");

        m_LED = new AddressableLED(cfgInt("LEDPort"));

        m_LEDBuffer = new AddressableLEDBuffer(cfgInt("LEDCount"));
        m_LED.setLength(cfgInt("LEDCount"));

        m_sections = new LEDSection[5];

        m_LED.start();

		m_stageMechanism = new Mechanism2d(1, 1);
		m_stageMechanism.setBackgroundColor(new Color8Bit(0, 0, 0));
		new TDSendable(this, "StageIndicator", "StageIndicator", m_stageMechanism);
		SmartDashboard.putData(m_stageMechanism);

		setSpeeds();
		
    }

    public static LED getInstance() {
        if (m_LEDLights == null) {
            m_LEDLights = new LED();
        }
        return m_LEDLights;
    }

	/**
	 * Sets the pattern on a given section to be drawn during the LED subsystem's
	 * periodic. Once the periodic completes, the patterns must be sent again,
	 * or the subsystem will just draw the fallback pattern.
	 * @param section Index of the section to draw on.
	 * @param pattern The new pattern to draw.
	 * @param priority The priority of the pattern, so patterns assigned later must have a
	 * higher priority in order to be drawn instead.
	 */
    public void setPattern(int section, LEDPattern pattern, LEDSection.Priority priority) {
        if (section < 0 || section >= m_sections.length) return;
		if (m_sections[section] == null || priority.greater(m_sections[section].getPriority())) {
        	m_sections[section] = new LEDSection(priority, pattern);
		}
    }

	/**
	 * Sets the pattern on a given section to be drawn during the LED subsystem's
	 * periodic. Once the periodic completes, the patterns must be sent again,
	 * or the subsystem will just draw the fallback pattern. Shorthand for
	 * `setPattern(section, pattern, Priority.BASIC)`.
	 * @param section Index of the section to draw on.
	 * @param pattern The new pattern to draw.
	 */
	public void setPattern(int section, LEDPattern pattern) {
		setPattern(section, pattern, LEDSection.Priority.BASIC);
	}



	/**
	 * Sets the pattern to be drawn if a section isn't assigned a pattern.
	 * This is typically handled by Robot.java.
	 */
	public void setFallbackPattern(LEDPattern pattern) {
		m_fallbackPattern = pattern;
	}

    public void gameStateLights() {
        FieldUtils.GameState gameState = FieldUtils.getInstance().getGameState();
		double stateTimeLeft = FieldUtils.getInstance().stateTimeLeft();
		//System.out.print("gamestate: ");
		//System.out.print(gameState);
		//System.out.print(" time left: ");
		//System.out.println(stateTimeLeft);
        if (stateTimeLeft < cfgDbl("stateChangeWarningTime")){
			setPattern(0, LEDPattern.kBlinkTeamColors, LEDSection.Priority.WARNING);
			setPattern(1, LEDPattern.kBlinkTeamColorsInverted, LEDSection.Priority.WARNING);
			setPattern(2, LEDPattern.kBlinkTeamColors, LEDSection.Priority.WARNING);
			setPattern(3, LEDPattern.kBlinkTeamColorsInverted, LEDSection.Priority.WARNING);
			setPattern(4, LEDPattern.kBlinkTeamColors, LEDSection.Priority.WARNING);
		}

		Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

		Color8Bit color = new Color8Bit();
		if (gameState == null) color = new Color8Bit(0,0,0);
		else {
			switch (gameState) {
				case AUTO:
					setPattern(0, LEDPattern.kSolidYellow, LEDSection.Priority.BASIC);
					setPattern(1, LEDPattern.kSolidYellow, LEDSection.Priority.BASIC);
					setPattern(2, LEDPattern.kSolidYellow, LEDSection.Priority.BASIC);
					setPattern(3, LEDPattern.kSolidYellow, LEDSection.Priority.BASIC);
					setPattern(4, LEDPattern.kSolidYellow, LEDSection.Priority.BASIC);
					color = new Color8Bit(0,0,0);
					break;
				case TRANSITION:
					setPattern(0, LEDPattern.kSolidLightBlue, LEDSection.Priority.BASIC);
					setPattern(1, LEDPattern.kSolidLightPink, LEDSection.Priority.BASIC);
					setPattern(2, LEDPattern.kSolidWhite, LEDSection.Priority.BASIC);
					setPattern(3, LEDPattern.kSolidLightPink, LEDSection.Priority.BASIC);
					setPattern(4, LEDPattern.kSolidLightBlue, LEDSection.Priority.BASIC);
					color = new Color8Bit(255,0,255);
					break;
				case RED_START:
					setPattern(0, LEDPattern.kSolidRed, LEDSection.Priority.BASIC);
					setPattern(1, LEDPattern.kSolidWhite, LEDSection.Priority.BASIC);
					setPattern(2, LEDPattern.kSolidRed, LEDSection.Priority.BASIC);
					setPattern(3, LEDPattern.kSolidWhite, LEDSection.Priority.BASIC);
					setPattern(4, LEDPattern.kSolidRed, LEDSection.Priority.BASIC);
					color = alliance == Alliance.Red ? new Color8Bit(0,255,0) : new Color8Bit(255,0,0);
					break;
				case BLUE_START:
					setPattern(0, LEDPattern.kSolidBlue, LEDSection.Priority.BASIC);
					setPattern(1, LEDPattern.kSolidWhite, LEDSection.Priority.BASIC);
					setPattern(2, LEDPattern.kSolidBlue, LEDSection.Priority.BASIC);
					setPattern(3, LEDPattern.kSolidWhite, LEDSection.Priority.BASIC);
					setPattern(4, LEDPattern.kSolidBlue, LEDSection.Priority.BASIC);
					color = alliance == Alliance.Blue ? new Color8Bit(0,255,0) : new Color8Bit(255,0,0);
					break;
				case ENDGAME:
					setPattern(0, LEDPattern.kSolidWhite, LEDSection.Priority.BASIC);
					color = new Color8Bit(0,255,0);
					break;
				default: 
					break;
			}
		}
		if (stateTimeLeft < cfgDbl("stateChangeWarningTime")) {
			color = ((int)(Timer.getFPGATimestamp()*4)%2 == 0 ? color : new Color8Bit(255, 255, 0));
		}
		m_stageMechanism.setBackgroundColor(color);
    }

	public void setSpeeds(){
		m_blinkSpeed = cfgInt("blinkSpeed");
		m_checkeredSpeed = cfgInt("checkeredSpeed");
		m_checkeredBlinkSpeed = cfgInt("checkeredBlinkSpeed");
		m_slideSpeed = cfgInt("slideSpeed");
		m_gradientSpeed = cfgInt("gradientSpeed");
		m_rainbowSpeed = cfgInt("rainbowSpeed");
	}

    @Override
    public void periodic() {
        int tick = (int)(Timer.getFPGATimestamp()*1000);
        for (int i = 0; i < m_sections.length; i++) {
			LEDPattern pattern;
			if (m_sections[i] != null) pattern = m_sections[i].getPattern();
			else if (m_fallbackPattern != null) pattern = m_fallbackPattern;
			else continue;

            int startIndex = (m_LEDBuffer.getLength()*i/m_sections.length);
            int endIndex = (m_LEDBuffer.getLength()*(i+1)/m_sections.length);

            for (int j = startIndex; j < endIndex; j++) {
                Color color = pattern.getColor(j, tick, m_LEDBuffer.getLength());
                m_LEDBuffer.setLED(j, color);
            }

			m_sections[i] = null;
        }

		gameStateLights();

        m_LED.setData(m_LEDBuffer);
        super.periodic();
    }
}
