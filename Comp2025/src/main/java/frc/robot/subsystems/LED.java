//
// LED class - LED feedback on robot
//
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LEDConsts.ANIMATION;
import frc.robot.Constants.LEDConsts.COLOR;
import frc.robot.Constants.Ports;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * LED class to control LED changes and provide command factory
 */
public class LED
{
  // Constants

  /*
   * Slot 0: LEDs 0 to 7 (0-7 are onboard, 8-399 are an external strip)
   * - CANdle supports 8 animation slots (0-7).
   */
  private static final int                     kSlot          = 0;    // Animation slot (0 - 7)
  private static final int                     kSlot0StartIdx = 0;    // LEDs controled by animation slot 0
  private static final int                     kSlot0EndIdx   = 7;

  private static final double                  kFrameRate     = 2.0;  // Animation speed in Hz 2-1000
  private static final double                  kBrightness    = 0.7;  // Brightness level 0.0 - 1.0
  private static final AnimationDirectionValue kDirection     = AnimationDirectionValue.Forward;
  private static final int                     kSize          = 4;    // Number of LEDs in animations that use partial string
  private static final double                  kSparking      = 0.4;  // Sparking level 0.0 - 1.0
  private static final double                  kCooling       = 0.5;  // Cooling level 0.0 - 1.0
  private static final double                  kMaxLEDsOn     = 0.5;  // Sparking level 0.0 - 1.0

  /*
   * LED object for managing control requests
   */
  private class LEDRequest
  {
    COLOR     color     = COLOR.OFF;
    ANIMATION animation = ANIMATION.SOLID;

    LEDRequest(COLOR reqColor, ANIMATION reqAnimation)
    {
      color = reqColor;
      animation = reqAnimation;
    }
  }

  /* Color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex */
  private static final RGBWColor           kWhite             = new RGBWColor(Color.kWhite).scaleBrightness(kBrightness);
  private static final RGBWColor           kRed               = new RGBWColor(Color.kRed).scaleBrightness(kBrightness);
  private static final RGBWColor           kOrange            = new RGBWColor(Color.kOrange).scaleBrightness(kBrightness);
  private static final RGBWColor           kYellow            = new RGBWColor(Color.kYellow).scaleBrightness(kBrightness);
  private static final RGBWColor           kGreen             = new RGBWColor(Color.kGreen).scaleBrightness(kBrightness);
  private static final RGBWColor           kBlue              = new RGBWColor(Color.kBlue).scaleBrightness(kBrightness);
  private static final RGBWColor           kViolet            = new RGBWColor(Color.kViolet).scaleBrightness(kBrightness);
  private static final RGBWColor           kOff               = new RGBWColor(0, 0, 0).scaleBrightness(kBrightness);

  // Member objects
  private final CANdle                     m_candle           = new CANdle(Ports.kCANID_CANdle);
  // private final CANdleSimState             m_candleSim        = new CANdleSimState(m_candle);
  private final SendableChooser<COLOR>     m_colorChooser     = new SendableChooser<COLOR>( );
  private final SendableChooser<ANIMATION> m_animationChooser = new SendableChooser<ANIMATION>( );

  private String                           m_name             = new String( );
  private LEDRequest                       m_request          = new LEDRequest(COLOR.OFF, ANIMATION.SOLID);
  private LEDRequest                       m_active           = new LEDRequest(COLOR.OFF, ANIMATION.SOLID);

  /****************************************************************************
   * 
   * Constructor
   */
  public LED( )
  {
    setName("LED");

    /* Configure CANdle */
    var cfg = new CANdleConfiguration( );

    /* set the LED strip type and brightness */
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.5;

    /* disable status LED when being controlled */
    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    cfg.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;

    m_candle.getConfigurator( ).apply(cfg);

    /* clear all previous animations */
    for (int i = 0; i < 8; ++i)
    {
      m_candle.setControl(new EmptyAnimation(i));
    }

    initDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Getter and setter for managing the class name
   */
  private void setName(String name)
  {
    m_name = name;
  }

  public String getName( )
  {
    return m_name;
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  public void periodic( )
  {
    // This method will be called once per scheduler run

    checkLEDRequests( );
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // StatusCode m_status = m_candleSim.getLastStatusCode( );
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
    SmartDashboard.putData("color", m_colorChooser);
    m_colorChooser.onChange(this::updateColorChooserCallback);

    // Animation options in Dashboard
    m_animationChooser.setDefaultOption("SOLID", ANIMATION.SOLID);
    m_animationChooser.addOption("COLORFLOW", ANIMATION.COLORFLOW);
    m_animationChooser.addOption("FIRE", ANIMATION.FIRE);
    m_animationChooser.addOption("LARSON", ANIMATION.LARSON);
    m_animationChooser.addOption("RAINBOW", ANIMATION.RAINBOW);
    m_animationChooser.addOption("RGBFADE", ANIMATION.RGBFADE);
    m_animationChooser.addOption("SINGLEFADE", ANIMATION.SINGLEFADE);
    m_animationChooser.addOption("STROBE", ANIMATION.STROBE);
    m_animationChooser.addOption("TWINKLE", ANIMATION.TWINKLE);
    m_animationChooser.addOption("TWINKLEOFF", ANIMATION.TWINKLEOFF);
    SmartDashboard.putData("animation", m_animationChooser);
    m_animationChooser.onChange(this::updateAnimationChooserCallback);
  }

  /****************************************************************************
   * 
   * Detect chooser changes and request new color and animation
   */
  private void updateColorChooserCallback(COLOR option)
  {
    setLEDs(option, m_animationChooser.getSelected( ));
  }

  private void updateAnimationChooserCallback(ANIMATION option)
  {
    setLEDs(m_colorChooser.getSelected( ), option);
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getName( )));
    setLEDs(COLOR.OFF, ANIMATION.SOLID);
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil6.getInstance( ).candlePrintFaults(m_candle, "CANdle");
    m_candle.clearStickyFaults( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set LEDs based on the requested color and animation
   */
  private void checkLEDRequests( )
  {
    RGBWColor ledColor;
    ControlRequest animation;

    if (m_request.color != m_active.color || m_request.animation != m_active.animation)
    {
      if (m_request.color == COLOR.DASHBOARD)
        m_request.color = m_colorChooser.getSelected( );

      if (m_request.animation == ANIMATION.DASHBOARD)
        m_request.animation = m_animationChooser.getSelected( );

      // Convert color request to a control request
      switch (m_request.color)
      {
        default :
        case OFF :
          ledColor = kOff;
          break;
        case WHITE :
          ledColor = kWhite;
          break;
        case RED :
          ledColor = kRed;
          break;
        case ORANGE :
          ledColor = kOrange;
          break;
        case YELLOW :
          ledColor = kYellow;
          break;
        case GREEN :
          ledColor = kGreen;
          break;
        case BLUE :
          ledColor = kBlue;
          break;
        case PURPLE :
          ledColor = kViolet;
          break;
      }

      // Convert animation request to a control request
      switch (m_request.animation)
      {
        default :
        case SOLID :
          animation = new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(ledColor);
          break;
        case COLORFLOW :
          animation = new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(kFrameRate)
              .withDirection(kDirection).withColor(ledColor);
          break;
        case FIRE :
          animation = new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(kFrameRate)
              .withDirection(kDirection).withBrightness(kBrightness).withCooling(kCooling).withSparking(kSparking);
          break;
        case LARSON :
          animation = new LarsonAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(kFrameRate).withSize(kSize)
              .withColor(ledColor);
          break;
        case RAINBOW :
          animation = new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(kFrameRate)
              .withDirection(kDirection).withBrightness(kBrightness);
          break;
        case RGBFADE :
          animation = new RgbFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(kFrameRate)
              .withBrightness(kBrightness);
          break;
        case SINGLEFADE :
          animation =
              new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(kFrameRate).withColor(ledColor);
          break;
        case STROBE :
          animation =
              new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(kFrameRate).withColor(ledColor);
          break;
        case TWINKLE :
          animation = new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(kFrameRate)
              .withMaxLEDsOnProportion(kMaxLEDsOn).withColor(ledColor);
          break;
        case TWINKLEOFF :
          animation = new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(kSlot).withFrameRate(kFrameRate)
              .withMaxLEDsOnProportion(kMaxLEDsOn).withColor(ledColor);
          break;
      }

      DataLogManager.log(String.format("%s: CANdle active now %s, %s", getName( ), m_request.color, m_request.animation));
      m_candle.setControl(animation);

      m_active.color = m_request.color;
      m_active.animation = m_request.animation;
    }
  }

  /****************************************************************************
   * 
   * Set LED requests based on the requested color and animation
   */
  private void setLEDs(COLOR color, ANIMATION animation)
  {
    m_request.color = color;
    m_request.animation = animation;
    DataLogManager.log(String.format("%s: CANdle request is %s, %s", getName( ), color, animation));
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
        ( ) -> setLEDs(color, animation) // Method to call
    )                                     //
        .withName("LEDSet")          //
        .ignoringDisable(true);
  }

}
