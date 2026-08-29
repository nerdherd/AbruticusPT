package frc.robot.commands;

import static frc.robot.Constants.SwerveDriveConstants.kDriveMaxVelocity;
import static frc.robot.Constants.SwerveDriveConstants.kDrivePrecisionMultiplier;
import static frc.robot.Constants.SwerveDriveConstants.kTurnMaxVelocity;
import static frc.robot.Constants.SwerveDriveConstants.kTurnPrecisionMultiplier;
import static frc.robot.Constants.SwerveDriveConstants.kTurnToAngleMaxVelocity;
import static frc.robot.Constants.ControllerConstants.kTranslationInputFilter;
import static frc.robot.Constants.ControllerConstants.kRotationInputFilter;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.util.NerdyMath;
public class 
SwerveJoystickCommand extends Command {
    private final NerdDrivetrain swerveDrive;
    private final Supplier<Double> xTranslationInput, yTranslationInput, turnInput, desiredAngle, usePrecisionMode;
    private final Supplier<Translation2d> robotOrientedAdjustment;
    private final Supplier<Boolean> useTurnToAngle, useFieldOriented, useTowMode;
    private final PIDController angleController;

    /**
     * Construct a new joystick command
     * @param swerveDrive
     * @param xTranslationInput - +forward
     * @param yTranslationInput - +right
     * @param turnInput - manual turning
     * @param useFieldOriented
     * @param useTowMode
     * @param usePrecisionMode
     * @param desiredAngleSupplier - rad, return 0.0 to use manual
     * @param robotOrientedAdjustment
     */
    public SwerveJoystickCommand(
            NerdDrivetrain swerveDrive,
            Supplier<Double> xTranslationInput, 
            Supplier<Double> yTranslationInput, 
            Supplier<Double> turnInput,
            Supplier<Boolean> useTurnToAngle,
            Supplier<Double> desiredAngleSupplier,
            Supplier<Translation2d> robotOrientedAdjustment,
            Supplier<Boolean> useFieldOriented, 
            Supplier<Boolean> useTowMode, 
            Supplier<Double> usePrecisionMode
        ) {
        this.swerveDrive = swerveDrive;

        this.xTranslationInput = xTranslationInput;
        this.yTranslationInput = yTranslationInput;
        this.turnInput = turnInput;
        this.useTurnToAngle = useTurnToAngle;
        this.desiredAngle = desiredAngleSupplier;

        this.robotOrientedAdjustment = robotOrientedAdjustment;

        this.useFieldOriented = useFieldOriented;
        this.useTowMode = useTowMode;
        this.usePrecisionMode = usePrecisionMode;

        this.angleController = new PIDController(
            SwerveDriveConstants.kTurnToAnglePIDConstants.kP,
            SwerveDriveConstants.kTurnToAnglePIDConstants.kI,
            SwerveDriveConstants.kTurnToAnglePIDConstants.kD
            );
        this.angleController.setTolerance(
            SwerveDriveConstants.kTurnToAngleTolerances.maxVelocity, 
            SwerveDriveConstants.kTurnToAngleTolerances.maxAcceleration
            );
        this.angleController.enableContinuousInput(-Math.PI, Math.PI);
    
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (useTowMode.get()) {
            swerveDrive.setControl(SwerveDriveConstants.kTowSwerveRequest);
            return;
        }
        
        /** m/s */
        Translation2d filteredInput = kTranslationInputFilter.apply(xTranslationInput.get(), yTranslationInput.get());
        double driveMult = NerdyMath.lerp(1.0, kDrivePrecisionMultiplier, usePrecisionMode.get());
        double xSpeed = filteredInput.getX() * kDriveMaxVelocity * driveMult;
        double ySpeed = filteredInput.getY() * kDriveMaxVelocity * driveMult;

        /** rad/s */
        double turnSpeed;

        if (useTurnToAngle.get()) {
            double targetAngle = desiredAngle.get();
            if (!Double.isNaN(targetAngle)) {
                turnSpeed = angleController.calculate(swerveDrive.getSwerveHeadingRadians(), targetAngle);
                turnSpeed = Math.min(kTurnToAngleMaxVelocity, Math.max(-kTurnToAngleMaxVelocity, turnSpeed));
            } else turnSpeed = 0.0;
        }
        else turnSpeed = kRotationInputFilter.apply(turnInput.get()) * kTurnMaxVelocity * NerdyMath.lerp(1.0, kTurnPrecisionMultiplier, usePrecisionMode.get());

        Translation2d adjustment = robotOrientedAdjustment.get();
        if (!adjustment.equals(Translation2d.kZero)) swerveDrive.driveRobotOriented(adjustment.getX() * driveMult, adjustment.getY() * driveMult, turnSpeed);
        else if (useFieldOriented.get()) swerveDrive.driveFieldOriented(xSpeed, ySpeed, turnSpeed);
        else swerveDrive.driveRobotOriented(xSpeed, ySpeed, turnSpeed);
    }
}