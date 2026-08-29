// VERSION 2.0
// Includes GuliKit.java v2.0
// Includes Xbox360Controller.java v1.0

package frc.robot.util.Controller;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// thank you william
// you're welcome mason
public class Controller {
    private CommandPS4Controller cmdPS4;
    private PS4Controller ps4;
    private CommandPS5Controller cmdPS5;
    private PS5Controller ps5;
    private XboxController xbox;
    private CommandXboxController cmdXbox;

    private final Type type;

    public enum Type {
        PS4,
        PS5,
        Xbox
    }

    /**
     * Constructor for PS4, PS5, Xbox, and Xbox360 controllers.<p>
     * Use {@link #Controller(int, Type, boolean, boolean)} for GuliKit controllers.
     * @param port in Driver Station
     * @param type of controller
     */
    public Controller(int port, Type type) {
        this.type = type;

        switch (type) {
            case PS4:
                cmdPS4 = new CommandPS4Controller(port);
                ps4 = cmdPS4.getHID();
                break;
            case PS5:
                cmdPS5 = new CommandPS5Controller(port);
                ps5 = cmdPS5.getHID();
                break;
            case Xbox:
                cmdXbox = new CommandXboxController(port);
                xbox = cmdXbox.getHID();
                break;
        }
    }

    /**
     * Constructor for a GuliKit controller.
     * @param port in Driver Station
     * @param type of controller (GuliKit)
     * @param isDigLeft discrete or continuous for the left joystick
     * @param isDigRight discrete or continuous for the right joystick
     */
    public Controller(int port, Type type, boolean isDigLeft, boolean isDigRight) {
        this.type = type;
    }

    /**
     * PS4 controller implemntation.<p>
     * Combines {@link CommandPS4Controller} and {@link PS4Controller}.
     * @param port The port of the controller on FRC Driver Station
     * @deprecated kept for previous versions
     */
    @Deprecated(forRemoval = false)
    public Controller(int port) {
        type = Type.PS4;

        cmdPS4 = new CommandPS4Controller(port);
        ps4 = cmdPS4.getHID();
    }

    /**
     * PS4/PS5 controller implemntation.<p>
     * Combines {@link CommandPS4Controller}, {@link PS4Controller}, {@link CommandPS5Controller}, and {@link PS5Controller}. 
     * @param port The port of the controller on FRC Driver Station
     * @param usePS4 Whether to use PS4 (true) or PS5 (false)
     * @deprecated kept for previous versions
     */
    @Deprecated(forRemoval = false)
    public Controller(int port, boolean usePS4) {
        if (usePS4) {
            type = Type.PS4;

            cmdPS4 = new CommandPS4Controller(port);
            ps4 = cmdPS4.getHID();
        } else {
            type = Type.PS5;

            cmdPS5 = new CommandPS5Controller(port);
            ps5 = cmdPS5.getHID();
        }
    }

    /**
     * {@link GuliKit} controller implementation.
     * @param port The port of the controller on FRC Driver Station
     * @param isDigLeft If the left switch on the controller is set to digital (dot)
     * @param isDigRight If the right switch on the controller is set to digital (dot)
     * @deprecated kept for previous versions
     */
    @Deprecated(forRemoval = false)
    

    // ***** BUTTON METHODS ***** //

    public double getLeftX()            { switch (type) {case PS4: return ps4.getLeftX();           case PS5: return ps5.getLeftX();            case Xbox: return xbox.getLeftX();                      default: return 0.0;} }
    public double getLeftY()            { switch (type) {case PS4: return ps4.getLeftY();           case PS5: return ps5.getLeftY();            case Xbox: return xbox.getLeftY();                      default: return 0.0;} }
    public double getRightX()           { switch (type) {case PS4: return ps4.getRightX();          case PS5: return ps5.getRightX();           case Xbox: return xbox.getRightX();                     default: return 0.0;} }
    public double getRightY()           { switch (type) {case PS4: return ps4.getRightY();          case PS5: return ps5.getRightY();           case Xbox: return xbox.getRightY();                     default: return 0.0;} }
    public boolean getTriggerLeft()     { switch (type) {case PS4: return ps4.getL2Button();        case PS5: return ps5.getL2Button();         case Xbox: return xbox.getLeftTriggerAxis() >= 0.65;    default: return false;} }
    public boolean getTriggerRight()    { switch (type) {case PS4: return ps4.getR2Button();        case PS5: return ps5.getR2Button();         case Xbox: return xbox.getRightTriggerAxis() >= 0.65;   default: return false;} }  
    public boolean getBumperLeft()      { switch (type) {case PS4: return ps4.getL1Button();        case PS5: return ps5.getL1Button();         case Xbox: return xbox.getLeftBumperButton();           default: return false;} }
    public boolean getBumperRight()     { switch (type) {case PS4: return ps4.getR1Button();        case PS5: return ps5.getR1Button();         case Xbox: return xbox.getRightBumperButton();          default: return false;} }    
    public boolean getButtonUp()        { switch (type) {case PS4: return ps4.getTriangleButton();  case PS5: return ps5.getTriangleButton();   case Xbox: return xbox.getXButton();                    default: return false;} }
    public boolean getButtonRight()     { switch (type) {case PS4: return ps4.getCircleButton();    case PS5: return ps5.getCircleButton();     case Xbox: return xbox.getAButton();                    default: return false;} }
    public boolean getButtonDown()      { switch (type) {case PS4: return ps4.getCrossButton();     case PS5: return ps5.getCrossButton();      case Xbox: return xbox.getBButton();                    default: return false;} }
    public boolean getButtonLeft()      { switch (type) {case PS4: return ps4.getSquareButton();    case PS5: return ps5.getSquareButton();     case Xbox: return xbox.getYButton();                    default: return false;} }
    public boolean getDpadUp()          { switch (type) {case PS4: return ps4.getPOV() == 0;        case PS5: return ps5.getPOV() == 0;         case Xbox: return xbox.getPOV() == 0;                   default: return false;} }
    public boolean getDpadRight()       { switch (type) {case PS4: return ps4.getPOV() == 90;       case PS5: return ps5.getPOV() == 90;        case Xbox: return xbox.getPOV() == 90;                  default: return false;} }
    public boolean getDpadDown()        { switch (type) {case PS4: return ps4.getPOV() == 180;      case PS5: return ps5.getPOV() == 180;       case Xbox: return xbox.getPOV() == 180;                 default: return false;} }
    public boolean getDpadLeft()        { switch (type) {case PS4: return ps4.getPOV() == 270;      case PS5: return ps5.getPOV() == 270;       case Xbox: return xbox.getPOV() == 270;                 default: return false;} }
    public boolean getJoystickLeft()    { switch (type) {case PS4: return ps4.getL3Button();        case PS5: return ps5.getL3Button();         case Xbox: return xbox.getLeftStickButton();            default: return false;} }
    public boolean getJoystickRight()   { switch (type) {case PS4: return ps4.getR3Button();        case PS5: return ps5.getR3Button();         case Xbox: return xbox.getRightStickButton();           default: return false;} }
    public boolean getControllerLeft()  { switch (type) {case PS4: return ps4.getShareButton();     case PS5: return ps5.getCreateButton();     case Xbox: return xbox.getStartButton();                default: return false;} }
    public boolean getControllerRight() { switch (type) {case PS4: return ps4.getOptionsButton();   case PS5: return ps5.getOptionsButton();    case Xbox: return xbox.getBackButton();                 default: return false;} }
    public Trigger triggerLeft()        { switch (type) {case PS4: return cmdPS4.L2();              case PS5: return cmdPS5.L2();               case Xbox: return cmdXbox.leftTrigger();                default: return null;} }
    public Trigger triggerRight()       { switch (type) {case PS4: return cmdPS4.R2();              case PS5: return cmdPS5.R2();               case Xbox: return cmdXbox.rightTrigger();               default: return null;} }
    public Trigger bumperLeft()         { switch (type) {case PS4: return cmdPS4.L1();              case PS5: return cmdPS5.L1();               case Xbox: return cmdXbox.leftBumper();                 default: return null;} }
    public Trigger bumperRight()        { switch (type) {case PS4: return cmdPS4.R1();              case PS5: return cmdPS5.R1();               case Xbox: return cmdXbox.rightBumper();                default: return null;} }
    public Trigger buttonUp()           { switch (type) {case PS4: return cmdPS4.triangle();        case PS5: return cmdPS5.triangle();         case Xbox: return cmdXbox.y();                          default: return null;} }
    public Trigger buttonRight()        { switch (type) {case PS4: return cmdPS4.circle();          case PS5: return cmdPS5.circle();           case Xbox: return cmdXbox.b();                          default: return null;} }
    public Trigger buttonDown()         { switch (type) {case PS4: return cmdPS4.cross();           case PS5: return cmdPS5.cross();            case Xbox: return cmdXbox.a();                          default: return null;} }
    public Trigger buttonLeft()         { switch (type) {case PS4: return cmdPS4.square();          case PS5: return cmdPS5.square();           case Xbox: return cmdXbox.x();                          default: return null;} }
    public Trigger dpadUp()             { switch (type) {case PS4: return cmdPS4.povUp();           case PS5: return cmdPS5.povUp();            case Xbox: return cmdXbox.povUp();                      default: return null;} }
    public Trigger dpadRight()          { switch (type) {case PS4: return cmdPS4.povRight();        case PS5: return cmdPS5.povRight();         case Xbox: return cmdXbox.povRight();                   default: return null;} }
    public Trigger dpadDown()           { switch (type) {case PS4: return cmdPS4.povDown();         case PS5: return cmdPS5.povDown();          case Xbox: return cmdXbox.povDown();                    default: return null;} }
    public Trigger dpadLeft()           { switch (type) {case PS4: return cmdPS4.povLeft();         case PS5: return cmdPS5.povLeft();          case Xbox: return cmdXbox.povLeft();                    default: return null;} }
    public Trigger joystickLeft()       { switch (type) {case PS4: return cmdPS4.L3();              case PS5: return cmdPS5.L3();               case Xbox: return cmdXbox.leftStick();                  default: return null;} }
    public Trigger joystickRight()      { switch (type) {case PS4: return cmdPS4.R3();              case PS5: return cmdPS5.R3();               case Xbox: return cmdXbox.rightStick();                 default: return null;} }
    public Trigger controllerLeft()     { switch (type) {case PS4: return cmdPS4.share();           case PS5: return cmdPS5.create();           case Xbox: return cmdXbox.start();                      default: return null;} }
    public Trigger controllerRight()    { switch (type) {case PS4: return cmdPS4.options();         case PS5: return cmdPS5.options();          case Xbox: return cmdXbox.back();                       default: return null;} }

    // ***** STATE METHODS ***** //

    /** <STRONG> GULIKIT ONLY </STRONG> */
    

    /**
     * returns analog value from left trigger
     * @return value on [0, 1]
     */
    public double getTriggerLeftAxis() {
        if (this.type == Type.PS4) {
            return ps4.getL2Axis();
        }
        return 0.0;
    }

    /**
     * returns analog value from right trigger
     * @return value on [0, 1]
     */
    public double getTriggerRightAxis() {
        if (this.type == Type.PS4) {
            return ps4.getR2Axis();
        }
        return 0.0;
    }
}