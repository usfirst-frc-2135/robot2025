//
// LED subystem - LED feedback on robot
//
package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConsts.ANIMATION;
import frc.robot.Constants.LEDConsts.COLOR;
import frc.robot.Constants.Ports;
import frc.robot.lib.phoenix.PhoenixUtil5;

/****************************************************************************
 * 
 * LED subsystem to control LED changes and provide command factory
 */
public class LED extends SubsystemBase
{
  // Constants
  private static final int                 kLEDCount           = 8;

  private final Color8Bit                  kWhite              = new Color8Bit(255, 255, 255);
  private final Color8Bit                  kRed                = new Color8Bit(255, 0, 0);
  private final Color8Bit                  kOrange             = new Color8Bit(255, 48, 0);
  private final Color8Bit                  kYellow             = new Color8Bit(255, 255, 0);
  private final Color8Bit                  kGreen              = new Color8Bit(0, 255, 0);
  private final Color8Bit                  kBlue               = new Color8Bit(0, 0, 255);
  private final Color8Bit                  kPurple             = new Color8Bit(128, 0, 128);
  private final Color8Bit                  kOff                = new Color8Bit(0, 0, 0);

  private final int                        kWhiteness          = 0;    // White level for LED strings that support one
  private final double                     kBrightness         = 0.7;  // Brightness level 0.0 - 1.0
  private final double                     kSpeed              = 0.5;  // Animation speed 0.0 - 1.0
  private final int                        kSlot               = 0;

  // Member objects
  private final CANdle                     m_candle            = new CANdle(Ports.kCANID_CANdle);
  private final SendableChooser<COLOR>     m_colorChooser      = new SendableChooser<COLOR>( );
  private final SendableChooser<ANIMATION> m_animationChooser  = new SendableChooser<ANIMATION>( );

  private Color8Bit                        m_color             = kOff;
  private Animation                        m_animation         = null;
  private Color8Bit                        m_previousColor     = kOff;
  private Animation                        m_previousAnimation = null;

  // Shuffleboard objects
  // private static final String              kLEDTab             = "LED";
  // ShuffleboardTab                          m_ledTab            = Shuffleboard.getTab(kLEDTab);
  // ShuffleboardLayout                       m_ledList           =
  //     m_ledTab.getLayout("LED", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
  // ComplexWidget                            m_colorWidget       = m_ledList.add("color", m_colorChooser);
  // ComplexWidget                            m_animationWidget   = m_ledList.add("animation", m_animationChooser);
  // ComplexWidget                            m_commandEntry      =
  //     m_ledList.add("LEDSet", this.getLEDCommand(COLOR.DASHBOARD, ANIMATION.DASHBOARD));

  /****************************************************************************
   * 
   * Constructor
   */
  public LED( )
  {
    setName("LED");
    setSubsystem("LED");

    m_candle.configBrightnessScalar(kBrightness);
    m_candle.clearAnimation(kSlot);

    initDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Initialize dashboard widgets

    // Add options for colors in Dashboard
    m_colorChooser.setDefaultOption("OFF", COLOR.OFF);
    m_colorChooser.addOption("WHITE", COLOR.WHITE);
    m_colorChooser.addOption("RED", COLOR.RED);
    m_colorChooser.addOption("ORANGE", COLOR.ORANGE);
    m_colorChooser.addOption("YELLOW", COLOR.YELLOW);
    m_colorChooser.addOption("GREEN", COLOR.GREEN);
    m_colorChooser.addOption("BLUE", COLOR.BLUE);
    m_colorChooser.addOption("PURPLE", COLOR.PURPLE);

    // Animation options in Dashboard
    m_animationChooser.setDefaultOption("CLEARALL", ANIMATION.CLEARALL);
    m_animationChooser.addOption("COLORFLOW", ANIMATION.COLORFLOW);
    m_animationChooser.addOption("FIRE", ANIMATION.FIRE);
    m_animationChooser.addOption("LARSON", ANIMATION.LARSON);
    m_animationChooser.addOption("RAINBOW", ANIMATION.RAINBOW);
    m_animationChooser.addOption("RGBFADE", ANIMATION.RGBFADE);
    m_animationChooser.addOption("SINGLEFADE", ANIMATION.SINGLEFADE);
    m_animationChooser.addOption("STROBE", ANIMATION.STROBE);
    m_animationChooser.addOption("TWINKLE", ANIMATION.TWINKLE);
    m_animationChooser.addOption("TWINKLEOFF", ANIMATION.TWINKLEOFF);
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setLEDs(COLOR.OFF, ANIMATION.CLEARALL);
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil5.getInstance( ).candlePrintFaults(m_candle, "candle");
    m_candle.clearStickyFaults( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set LEDs based on the color and animation requested
   * 
   * @param color
   *          requested color
   * @param animation
   *          requested animation
   */
  private void setLEDs(COLOR color, ANIMATION animation)
  {
    if (color == COLOR.DASHBOARD)
      color = m_colorChooser.getSelected( );

    if (animation == ANIMATION.DASHBOARD)
      animation = m_animationChooser.getSelected( );

    switch (color)
    {
      default :
      case OFF :
        m_color = kOff;
        break;
      case WHITE :
        m_color = kWhite;
        break;
      case RED :
        m_color = kRed;
        break;
      case ORANGE :
        m_color = kOrange;
        break;
      case YELLOW :
        m_color = kYellow;
        break;
      case GREEN :
        m_color = kGreen;
        break;
      case BLUE :
        m_color = kBlue;
        break;
      case PURPLE :
        m_color = kPurple;
        break;
    }

    switch (animation)
    {
      default :
      case CLEARALL :
        m_animation = null;
        break;
      case COLORFLOW :
        m_animation =
            new ColorFlowAnimation(m_color.red, m_color.green, m_color.blue, kWhiteness, kSpeed, kLEDCount, Direction.Forward);
        break;
      case FIRE :
        m_animation = new FireAnimation(kBrightness, kSpeed, kLEDCount, 0.7, 0.5);
        break;
      case LARSON :
        m_animation =
            new LarsonAnimation(m_color.red, m_color.green, m_color.blue, kWhiteness, kSpeed, kLEDCount, BounceMode.Front, 3);
        break;
      case RAINBOW :
        m_animation = new RainbowAnimation(kBrightness, kSpeed, kLEDCount);
        break;
      case RGBFADE :
        m_animation = new RgbFadeAnimation(kBrightness, kSpeed, kLEDCount);
        break;
      case SINGLEFADE :
        m_animation = new SingleFadeAnimation(m_color.red, m_color.green, m_color.blue, kWhiteness, kSpeed, kLEDCount);
        break;
      case STROBE :
        m_animation = new StrobeAnimation(m_color.red, m_color.green, m_color.blue, kWhiteness, kSpeed, kLEDCount);
        break;
      case TWINKLE :
        m_animation = new TwinkleAnimation(m_color.red, m_color.green, m_color.blue, kWhiteness, kSpeed, kLEDCount,
            TwinklePercent.Percent64);
        break;
      case TWINKLEOFF :
        m_animation = new TwinkleOffAnimation(m_color.red, m_color.green, m_color.blue, kWhiteness, kSpeed, kLEDCount,
            TwinkleOffPercent.Percent100);
        break;
    }

    if (m_color == m_previousColor && m_animation == m_previousAnimation)
    {
      DataLogManager.log(String.format("%s: Color and animation already active - %s, %s", getSubsystem( ), color, animation));
      return;
    }

    DataLogManager.log(String.format("%s: Color is now %s, %s", getSubsystem( ), color, animation));
    if (m_animation != null)
    {
      m_candle.animate(m_animation, kSlot);
    }
    else
    {
      m_candle.clearAnimation(kSlot);
      m_candle.setLEDs(m_color.red, m_color.green, m_color.blue, kWhiteness, 0, kLEDCount);
    }

    m_previousColor = m_color;
    m_previousAnimation = m_animation;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create LED set command
   * 
   * @param color
   *          LED color to display
   * @param animation
   *          LED animation pattern to use
   * @return instant command that changes LEDs
   */
  public Command getLEDCommand(COLOR color, ANIMATION animation)
  {
    return new InstantCommand(            // Command that runs exactly once
        ( ) -> setLEDs(color, animation), // Method to call
        this                              // Subsystem requirement
    )                                     //
        .withName("LEDSet")          //
        .ignoringDisable(true);
  }

}
