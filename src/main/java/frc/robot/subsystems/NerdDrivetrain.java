// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.PathPlannerConstants.kPPRotationPIDConstants;
import static frc.robot.Constants.PathPlannerConstants.kPPTranslationPIDConstants;
import static frc.robot.Constants.SwerveDriveConstants.kApplyRobotSpeedsRequest;
import static frc.robot.Constants.SwerveDriveConstants.kFieldOrientedSwerveRequest;
import static frc.robot.Constants.SwerveDriveConstants.kRobotOrientedSwerveRequest;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveController;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveMaxLateralVelocity;
import static frc.robot.Constants.SwerveDriveConstants.kTowSwerveRequest;

import java.util.HashMap;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
// import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.NerdyMath;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable.LOG_LEVEL;
// import frc.robot.util.logging.NerdLog;
// import frc.robot.util.logging.Reportable;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

public class NerdDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Sendable{
    private static final boolean USE_VISION = false;
    public final Field2d field;  
    public boolean useMegaTag2 = false;
    
    
    public NerdDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        RobotConfig robotConfig = null;
        try {
        robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        e.printStackTrace();
        }

        AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        (speeds,feedforwards) -> setControl(
            kApplyRobotSpeedsRequest.withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),
        new PPHolonomicDriveController(
            kPPTranslationPIDConstants,
            kPPRotationPIDConstants),
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? (alliance.get() == DriverStation.Alliance.Red) : false;
            },
            this        
        );

        field = new Field2d();

       // setVision(USE_VISION);

    }

    @Override
    public void periodic() {
        field.setRobotPose(getPose());

        // if (USE_VISION) {
        //     visionUpdate(Camera.Front, true);
        //     visionUpdate(Camera.Back, DriverStation.isTeleop());
        // }
    }

    public void driveFieldOriented(double xSpeed, double ySpeed, double rSpeed) {
        setControl(kFieldOrientedSwerveRequest
        .withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(rSpeed) 
        );
    }

    public void driveRobotOriented(double xSpeed, double ySpeed, double rSpeed) {
        setControl(kRobotOrientedSwerveRequest
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rSpeed)
        );
    }

    public void resetTargetDrive() {
        kTargetDriveController.reset("x", getPose().getX(), getFieldOrientedSpeeds().vxMetersPerSecond * 0.1);
        kTargetDriveController.reset("y", getPose().getY(), getFieldOrientedSpeeds().vyMetersPerSecond * 0.1);
        kTargetDriveController.reset("r", getSwerveHeadingRadians(), getFieldOrientedSpeeds().omegaRadiansPerSecond * 0.1);     
    }

    public void tow() {
        setControl(kTowSwerveRequest);
    }

    public void stop() {
        driveFieldOriented(0.0, 0.0, 0.0);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }


    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    public ChassisSpeeds getFieldOrientedSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), Rotation2d.fromDegrees(getSwerveHeadingDegrees()));
    }

    public void setBrake(boolean brake) {
        configNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public double angleToPose(FieldPositions pos) {
        return NerdyMath.angleToPose(getPose(), pos.get());
    }


    private final double deviceTempThreshold = 50;
    /** @return "it's chill" unless the temperature of any motor is above the threshold, reports id and type */
    public String pollTemperatures() {
        String output = "";
        for (int i = 0; i < 4; i++) {
            double driveTemp = getModule(i).getDriveMotor().getDeviceTemp().getValueAsDouble();
            double steerTemp = getModule(i).getSteerMotor().getDeviceTemp().getValueAsDouble();
            if (driveTemp >= deviceTempThreshold) output += "Drive " + i + ", Temp: " + driveTemp;
            if (steerTemp >= deviceTempThreshold) output += "Steer " + i + ", Temp: " + steerTemp;
            
        }
        if (!output.equals("")) return output;
        return "it's chill";
    }

    private double pollStatorCurrentSum() {
        double sum = 0;
        for (int i = 0; i < 4; i++) {
            sum += Math.abs(getModule(i).getDriveMotor().getTorqueCurrent().getValueAsDouble());
            sum += Math.abs(getModule(i).getSteerMotor().getTorqueCurrent().getValueAsDouble());
        }
        return sum;
    }



    /**
     * activates or deactivates vision by setting the pipeline either to 0 for active or 1 for inactive
     * and by adjusting throttle, see {@link LimelightHelpers#SetThrottle(String, int)}
     * @param activate whether to activate or deactivate
     */
    // public void setVision(boolean activate) {
    //     for (Camera camera : Camera.values()) {
    //         LimelightHelpers.setPipelineIndex(camera.name, (activate) ? 0 : 0);
    //         LimelightHelpers.SetThrottle(camera.name, (activate) ? 0 : 0);
    //     }
    // }
    
    /**
     * temporarily switch to megatag1 to update robot field heading/pose
     * @param delay in seconds until switching back to megatag2
     */
    public Command resetPoseWithAprilTags(double delay) {
        return Commands.sequence(
            Commands.runOnce(() -> useMegaTag2 = false),
            Commands.waitSeconds(delay),
            Commands.runOnce(() -> useMegaTag2 = true)
      );
    }

    // private HashMap<Camera, Double> lastTimestamps = new HashMap<>();    
    // public void visionUpdate(Camera limelight, boolean useReset) {
    //     if (LimelightHelpers.getCurrentPipelineIndex(limelight.name) != 0) return;
    //     if (!useMegaTag2) {
    //         // --------- MT1 --------- //
    //         if (!useReset) return;
    //         PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name);
    //         if (mt == null || Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720 || mt.tagCount == 0 || mt.avgTagDist >= 3.0) return;
    //         resetRotation(mt.pose.getRotation());
    //         useMegaTag2 = true;
    //         setDriverHeadingForward();
    //     }
    //     else {
    //         // --------- MT2 --------- //
    //         double yaw = getSwerveHeadingDegrees();
    //         LimelightHelpers.SetRobotOrientation(limelight.name, yaw, 0, 0, 0, 0, 0);
    //         PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name);
    //         field.getObject(limelight.name).setPose(nullPose);
    //         if (mt == null || Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720 || mt.tagCount == 0) return;
    //         // if (!lastTimestamps.containsKey(limelight)) lastTimestamps.put(limelight, 0.0);
    //         // if (lastTimestamps.get(limelight).equals(mt.timestampSeconds)) return;
    //         // lastTimestamps.put(limelight, mt.timestampSeconds);
    //         field.getObject(limelight.name).setPose(mt.pose);
    //         double stddev = (mt.avgTagDist > 3) ? 2.0 : 0.7;
    //         setVisionMeasurementStdDevs(VecBuilder.fill(stddev, stddev, 9999999)); // TODO consider other stddevs
    //         addVisionMeasurement(mt.pose, Utils.getCurrentTimeSeconds());
    //     }
    // }

    public void recalibrateGyroMT1() {
        resetRotation((RobotContainer.IsRedSide()) ? Rotation2d.k180deg : Rotation2d.kZero);
        useMegaTag2 = false;
    }

    /** 
     * Set the operator heading to forward based on alliance field forward
     * @see {@link #setOperatorPerspectiveForward} also for more custom setting
     */
    public void setDriverHeadingForward() {
        setOperatorPerspectiveForward(RobotContainer.IsRedSide() ? Rotation2d.k180deg : Rotation2d.kZero);
    }

    /** 
     * Set the operator heading to forward based on robot
     * @see {@link #setOperatorPerspectiveForward} also for more custom setting
     */
    public void setRobotHeadingForward() {
        setOperatorPerspectiveForward(getPose().getRotation());
    }
    
    /**
     * Get heading relative to what the operator sees in degrees
     * @see {@link #setDriverHeadingForward()} for resetting to zero
     */
    public double getDriverHeadingDegrees() {
        return getOperatorForwardDirection().getDegrees() + getSwerveHeadingDegrees();
    }

    /**
     * Get heading relative to what the operator sees in radians
     * @see {@link #setDriverHeadingForward()} for resetting to zero
     */
    public double getDriverHeadingRadians() {
        return getOperatorForwardDirection().getRadians() + getSwerveHeadingRadians();
    }

    /** 
     * Get absolute heading in degrees, from blue alliance orientation
     * @see {@link #resetRotation(Rotation2d)}
     */
    public double getSwerveHeadingDegrees() {
        return MathUtil.inputModulus(getPose().getRotation().getDegrees(), -180, 180);
    }

    /** 
     * Get absolute heading in radians, from blue alliance orientation
     * @see {@link #resetRotation(Rotation2d)}
     */
    public double getSwerveHeadingRadians() {
        return MathUtil.inputModulus(getPose().getRotation().getRadians(), -Math.PI, Math.PI);
    }

    // @Override
    // public void initializeLogging() {
    //     NerdLog.get().logData(kSwerveTab + "/Robot Field", field, LOG_LEVEL.MINIMAL);

    //     ///////////
    //     /// ALL ///
    //     ///////////
    //     NerdLog.get().logData(kSwerveTab + "/Commands", this, LOG_LEVEL.ALL);
    //     if (Constants.ROBOT_LOG_LEVEL == LOG_LEVEL.ALL) {
    //         Field2d positionField = new Field2d();
    //         for (FieldPositions position : FieldPositions.values()) {
    //             positionField.getObject(position.name() + "-blue").setPose(position.blue);
    //             positionField.getObject(position.name() + "-red").setPose(position.red);
    //         }
    //         NerdLog.get().logData(kSwerveTab +"/Object Field", positionField, LOG_LEVEL.ALL);
    //     }
    //     for (Camera camera : Camera.values())
    //         NerdLog.get().logBoolean(kSwerveTab + "/" + camera.name + " detecting", () -> LimelightHelpers.getTV(camera.name), LOG_LEVEL.ALL);

    //     NerdLog.get().logStructSerializable(kSwerveTab + "/Field Chassis Speeds", () -> getFieldOrientedSpeeds(), LOG_LEVEL.ALL);
    //     NerdLog.get().logSwerveModules(kSwerveTab + "/Swerve Module States", this::getState, LOG_LEVEL.ALL);

    //     //////////////
    //     /// MEDIUM ///
    //     //////////////
    //     NerdLog.get().logNumber(kSwerveTab + "/Swerve Heading", this::getSwerveHeadingDegrees, "deg", LOG_LEVEL.MEDIUM);
    //     NerdLog.get().logNumber(kSwerveTab + "/Driver Heading", this::getDriverHeadingDegrees, "deg", LOG_LEVEL.MEDIUM);
    //     NerdLog.get().logBoolean(kSwerveTab + "/Using MT2", () -> this.useMegaTag2, LOG_LEVEL.MEDIUM);
        
    //     //////////////
    //     /// MINIMAL //
    //     //////////////
    //     for (int i = 0; i < 4; i++) {
    //         NerdLog.get().logSignal(kSwerveTab + "/Temperatures/Drive " + i, getModule(i).getDriveMotor().getDeviceTemp(false), getModule(i).getDriveMotor().getNetwork().getName(), LOG_LEVEL.MINIMAL);
    //         NerdLog.get().logSignal(kSwerveTab + "/Temperatures/Turn " + i, getModule(i).getSteerMotor().getDeviceTemp(false), getModule(i).getSteerMotor().getNetwork().getName(), LOG_LEVEL.MINIMAL);
    //         NerdLog.getNT().logBoolean(kSwerveTab + "/Connected/Drive " + i, getModule(i).getDriveMotor()::isConnected, LOG_LEVEL.MINIMAL);
    //         NerdLog.getNT().logBoolean(kSwerveTab + "/Connected/Turn " + i, getModule(i).getSteerMotor()::isConnected, LOG_LEVEL.MINIMAL);
    //     }
    //     NerdLog.get().logNumber(kSwerveTab +"/Stator Current Sum", this::pollStatorCurrentSum, "A", LOG_LEVEL.MINIMAL);
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        builder.addStringProperty(
            ".default",
            () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
            null);
        builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
        builder.addStringProperty(
            ".command",
            () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
            null);
  }
}

