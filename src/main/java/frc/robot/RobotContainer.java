// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.util.NerdyMath;
import frc.robot.util.Controller.Controller;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static boolean isRedSide = false;

  private SwerveJoystickCommand swerveJoystickCommand;

  public NerdDrivetrain nerdDrivetrain;
  public final Controller driverController = new Controller(0);


  public static void refreshAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      isRedSide = (alliance.get() == DriverStation.Alliance.Red);
  }

  public static boolean IsRedSide() {
    return isRedSide;
  }

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandPS4Controller m_driverController =
  //     new CommandPS4Controller(ControllerConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // configureBindings();
    nerdDrivetrain = TunerConstants.createDrivetrain();
    initDefaultCommands_teleop();
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  // private void configureBindings() {
  //   // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  //   new Trigger(m_exampleSubsystem::exampleCondition)
  //       .onTrue(new ExampleCommand(m_exampleSubsystem));

  //   // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
  //   // cancelling on release.
  //   m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }



  public void initDefaultCommands_teleop() {
    

      swerveJoystickCommand = new SwerveJoystickCommand(
        nerdDrivetrain,
        // Horizontal Translation
        () -> -driverController.getLeftY(), 
        // Vertical Translation
        () -> -driverController.getLeftX(), 
        // Turn
        () -> -driverController.getRightX(), 
        // use turn to angle
        () -> false, // driverController.getBumperRight() || driverController.getBumperLeft(),
        // turn to angle target direction, 0.0 to use manual
        () -> 0.0, // (driverController.getBumperRight()) ? swerveDrive.angleToPose(FieldPositions.HUB_CENTER) : 0.0,
        // robot oriented adjustment (dpad)
        () -> new Translation2d(
          (driverController.getDpadUp() ? 1 : 0) - (driverController.getDpadDown() ? 1 : 0), 
          (driverController.getDpadLeft() ? 1 : 0) - (driverController.getDpadRight() ? 1 : 0)),
        // joystick drive field oriented
        () -> true, 
        // tow supplier
        () -> false, 
        // precision/programmer mode :)
        () -> driverController.getTriggerLeftAxis()
      );

      // new SwerveJoystickCommand(
      //   swerveDrive,
      //   () -> -commandDriverController.getLeftY(), // Horizontal translation
      //   commandDriverController::getLeftX, // Vertical Translation
      //   // () -> 0.0, // debug
      //   () -> {
      //     // if (driverController.getL2Button()) {
      //     //   SmartDashboard.putBoolean("Turn to angle 2", true);
      //     //   double turnPower = apriltagCamera.getTurnToTagPower(swerveDrive, angleError, IsRedSide() ? 4 : 7, adjustmentCamera); 
      //     //   SmartDashboard.putNumber("Turn Power", turnPower);
      //     //   return turnPower;
      //     // }
      //     // SmartDashboard.putBoolean("Turn to angle 2", false);
      //     if (swerveDrive.getTurnToAngleMode()) {
      //       return 0.0;
      //     }
      //     return commandDriverController.getRightX(); // Rotation
      //   },

      //   // driverController::getSquareButton, // Field oriented
      //   () -> false, // should be robot oriented now on true
      //   () -> false,
      //   // driverController::getCrossButton, // Towing
      //   // driverController::getR2Button, // Precision/"Sniper Button"
      //   () -> driverController.getR2Button(), // Precision mode (disabled)
      //   () -> {
      //     if (swerveDrive.getTurnToAngleMode()) {
      //       return (
      //       Math.abs(driverController.getRightX()) > 0.05 
      //       || Math.abs(driverController.getRightY()) > 0.05
      //       || driverController.getCircleButton()
      //       );
      //     } 
      //     else if (driverController.getCircleButton()) {
      //       return(true);  
      //     }
      //     else {
      //       return(false);
      //     }
      //   }, 
      //   // () -> false, // Turn to angle (disabled)
      //   () -> { // Turn To angle Direction
      //     if (driverController.getCircleButton()) { //turn to amp
      //       if (!IsRedSide()){
      //         return 270.0;
      //       }
      //       return 90.0;
      //     } 
      //     else {
      //       double xValue = commandDriverController.getRightX();
      //       double yValue = commandDriverController.getRightY();
      //       double magnitude = Math.sqrt((xValue*xValue) + (yValue*yValue));
      //       if (magnitude > 0.49) {
      //         double angle = (90 + NerdyMath.radiansToDegrees(Math.atan2(commandDriverController.getRightY(), commandDriverController.getRightX())));
      //         angle = (((-1 * angle) % 360) + 360) % 360;
      //         SmartDashboard.putNumber("desired angle", angle);
      //         return angle;
      //       }
      //       return 1000.0;
      //     }
      //   });

      nerdDrivetrain.setDefaultCommand(swerveJoystickCommand);

      DogLog.log("Intiialized teleop commands", "yeag");
  }

}
