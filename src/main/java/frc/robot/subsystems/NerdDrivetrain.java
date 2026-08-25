public class NerdDrivetrain {
    public final Field2 field;  
    public boolean useMegaTag2 = faslse;
    
    
    public NerdDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(drivetrainConstants, modules);

      RobotConfig RobotConfig = null;
      try {
        robotConfig = RobotConFig.fromGUISettings();
      } catch (Exception e) {
        e.printStackTrace();
      }

      AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        (speeds,feedforwards) -> setControl(
            kApplyRobotSpeedsRequest.withSpeeds(speeds)
            .withWheelForceFeedForwardsX(feedforwards.robotRelativeForceXNewtons())
            .withWheelForceFeedForwardsY(feedforwards.robotRelativeForceYNewtons())
        ),
        new PPHolonomicDriveController(
            kPPTranslationPIDConstants,
            kPPRotationPIDConstants),
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance,isPresent() ? (alliance,get() == DriverStation.Alliance.Red) : false;
            },
            this        
        );
        field = new Field2d();

        setVision(USE_VISION);

    }

    @Ovveride
    public void periodic() {
        field.setRobotPose(getPose());
        field.getObject("Look Ahead Ring Drive").setPose(getLookAheadPose(ShooterConstants.kLookAheadRingDriveFactor));
        field.getObject("Look Ahead").setPose(getLookAheadPose(ShooterConstants.kLookAheadRingDriveFactor));

        if (USE_VISION) {
            visionUpdate(Camera.Front, true);
            visionUpdate(Camera.Back, DriverStation.isTeleop());
        }
    }

    public void driveFieldOriented(double xSpeed, double ySpeed, double rSpeed) {
        setControl(kFieldOrientedSwerveRequest
        .withVeloctyX(xSpeed)
        .withVelocity(ySpeed)
        .withRotationRate(rSpeed)
        );
    }

    public void resetTargetDrive() {
        kTargetDriveController.reset("x", getPose().getX(), getFieldOrientedSpeeds().vxMetersPerSecond * 0.1);
        kTargetDriveController.reset("y", getPose().getY(), getFieldOrientedSpeeds().vYMetersPerSecond * 0.1);
        kTargetDriveController.reset("r", getSwerveHeadingRadians(), getFieldOrientedSpeeds().omegaRadiansPerSecond * 0.1);     
    }
    
}
