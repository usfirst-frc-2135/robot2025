//
// Swerve Subystem - command-based swerve subsystem
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.VIConsts;
import frc.robot.Robot;
import frc.robot.commands.LogCommand;
import frc.robot.commands.SwervePIDController;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.lib.LimelightHelpers;

// @formatter:off

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final boolean        m_useLimelight           = true;

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance  ntInst                   = NetworkTableInstance.getDefault( );

    /* Robot pose for field positioning */
    private final NetworkTable          table                    = ntInst.getTable("Pose");
    private final DoubleArrayPublisher  fieldPubLeft             = table.getDoubleArrayTopic("llPose-left").publish( );
    private final DoubleArrayPublisher  fieldPubRight            = table.getDoubleArrayTopic("llPose-right").publish( );
    private final StringPublisher       fieldTypePub             = table.getStringTopic(".type").publish( );

    private final NetworkTable              dsTable              = ntInst.getTable("DriveState");
    private final StructSubscriber<Pose2d>  drivePose            = dsTable.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d( ));

    /* Robot pose */

    // Network tables publisher objects
    private DoubleEntry                 poseXEntry;
    private DoubleEntry                 poseYEntry;
    private DoubleEntry                 poseRotEntry;
    private final NetworkTable          robotTable = ntInst.getTable(Constants.kRobotString);
    private IntegerSubscriber           reefLevel = robotTable.getIntegerTopic(ELConsts.kReefLevelString).subscribe((0));
    private IntegerSubscriber           reefBranch = robotTable.getIntegerTopic(VIConsts.kReefBranchString).subscribe((0));

    /* Robot pathToPose constraints */
    private final PathConstraints       kPathFindConstraints     = new PathConstraints( // 
        3.5,            // kMaxVelocityMps                               (slowed from 3.0 for testing)    
        3.5,      // kMaxAccelerationMpsSq                         (slowed from 3.0 for testing)  
        2.0 * Math.PI,                 // kMaxAngularSpeedRadiansPerSecond 
        2.0 * Math.PI                  // kMaxAngularSpeedRadiansPerSecondSquared 
    );

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
        initDashboard();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
        initDashboard();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
        initDashboard();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        if (m_useLimelight && Robot.isReal( )) {
            visionUpdate(Constants.kLLLeftName, fieldPubLeft);
            visionUpdate(Constants.kLLRightName, fieldPubRight);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    // @formatter:on

    /***********************************************************************************/
    /***********************************************************************************/

    /**
     * Add dashboard widgets and network table entries
     */
    private void initDashboard( )
    {
        // Get the default instance of NetworkTables that was created automatically when the robot program starts
        NetworkTable table = ntInst.getTable("swerve");

        poseXEntry = table.getDoubleTopic("X").getEntry(0.0);
        poseYEntry = table.getDoubleTopic("Y").getEntry(0.0);
        poseRotEntry = table.getDoubleTopic("rotation").getEntry(0.0);
        SmartDashboard.putData("SetPose", new InstantCommand(( ) -> setOdometryFromDashboard( ), this).ignoringDisable(true));

        SmartDashboard.putData("GetAlignToReefCommand", new DeferredCommand(( ) -> getAlignToReefCommand( ), Set.of(this)));
        SmartDashboard.putData("GetAlignToReefCommand2", new DeferredCommand(( ) -> getAlignToReefCommand2( ), Set.of(this)));
        SmartDashboard.putData("GetAlignToReefCommand3", SwervePIDController.generateCommand(this, Seconds.of(2.5)));

        SmartDashboard.putData("SetRobotPose", new InstantCommand( //
                ( ) ->
                {
                    Pose2d mt1Pos = new Pose2d(
                            new Translation2d(SmartDashboard.getNumber("mt1_X", 0), SmartDashboard.getNumber("mt1_Y", 0)),
                            new Rotation2d(Math.toRadians(SmartDashboard.getNumber("mt1_Rot", 0))));
                    resetPose(mt1Pos);
                    DataLogManager.log(String.format("SetRobotPose: Set pose to Megatag1 pose: %s", mt1Pos));

                }, this).ignoringDisable(true));

        // SmartDashboard.putData("CommandTest", test( ));
    }

    /**
     * Command for
     * 
     * testing alignment commands
     */
    // public Command test( )
    // {
    //     return new SequentialCommandGroup(                     //
    //             SwervePIDController.generateCommand(this, Seconds.of(2.5)), new InstantCommand(                     //  1
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(9.0, 1.0), new Rotation2d(Rotations.of(0.0)))), this),                          //
    //             getAlignToReefCommand3( ),              //
    //             new WaitCommand(0.25),                   //
    //             new InstantCommand(                     //  2
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(9.0, 4.0), new Rotation2d(Rotations.of(0.5)))), this),                          //
    //             getAlignToReefCommand3( ),              //
    //             new WaitCommand(0.25),                   //
    //             new InstantCommand(                     //  3
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(9.0, 7.5), new Rotation2d(Rotations.of(0.0)))), this),                          //
    //             getAlignToReefCommand3( ),              //
    //             new WaitCommand(0.25),                   //
    //             new InstantCommand(                     //  4
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(16.0, 1.0), new Rotation2d(Rotations.of(0.5)))), this),                          //
    //             getAlignToReefCommand3( ),              //
    //             new WaitCommand(0.25),                   //
    //             new InstantCommand(                     //  5
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(16.0, 4.0), new Rotation2d(Rotations.of(0.0)))), this),                          //
    //             getAlignToReefCommand3( ),              //
    //             new WaitCommand(0.25),                   //
    //             new InstantCommand(                     //  6
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(16.0, 7.5), new Rotation2d(Rotations.of(0.5)))), this),                          //
    //             getAlignToReefCommand3( ),              //
    //             new WaitCommand(0.25),                   //
    //             new InstantCommand(                     //  7
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(8.0, 2.0), new Rotation2d(Rotations.of(0.0)))), this),                          //
    //             getAlignToReefCommand3( ),              //
    //             new WaitCommand(0.25),                   //
    //             new InstantCommand(                     //  8
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(8.0, 7.5), new Rotation2d(Rotations.of(0.5)))), this),                          //
    //             getAlignToReefCommand3( ),              //
    //             new WaitCommand(0.25),                   //
    //             new InstantCommand(                     //  9
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(15.0, 2.0), new Rotation2d(Rotations.of(0.0)))), this),                          //
    //             getAlignToReefCommand3( ),              //
    //             new WaitCommand(0.25),                   //
    //             new InstantCommand(                     //  10
    //                     ( ) -> resetPose(new Pose2d(new Translation2d(15.0, 7.5), new Rotation2d(Rotations.of(0.5)))), this),                          //
    //             getAlignToReefCommand3( )              //
    //     );                                               //
    // }

    /**
     * Construct a path following commandand
     */
    public Command getPathCommand(PathPlannerPath ppPath)
    {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(ppPath).withName("swervePPPath");
    }

    /**
     * Limelight MegaTag example code for vision processing
     *
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to goal
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact
     * implementation of how to use vision should be tuned per-robot and to the team's specification.
     */
    private void visionUpdate(String limelightName, DoubleArrayPublisher fieldPub)
    {
        boolean useMegaTag2 = DriverStation.isEnabled( ); // set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if (useMegaTag2 == false)
        {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

            if (mt1 == null)
            {
                doRejectUpdate = true;
            }
            else
            {
                if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
                {
                    if (mt1.rawFiducials[0].ambiguity > .7)
                    {
                        doRejectUpdate = true;
                    }
                    if (mt1.rawFiducials[0].distToCamera > 3)
                    {
                        doRejectUpdate = true;
                    }
                }
                if (mt1.tagCount == 0)
                {
                    doRejectUpdate = true;
                }
            }

            if (!doRejectUpdate)
            {
                fieldTypePub.set("Field2d");
                fieldPub.set(new double[ ]
                {
                        mt1.pose.getX( ), mt1.pose.getY( ), mt1.pose.getRotation( ).getDegrees( )
                });
                setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                addVisionMeasurement(mt1.pose, mt1.timestampSeconds);

                SmartDashboard.putNumber("mt1X", mt1.pose.getX( ));
                SmartDashboard.putNumber("mt1Y", mt1.pose.getY( ));
                SmartDashboard.putNumber("mt1Rot", mt1.pose.getRotation( ).getDegrees( ));
            }

        }
        else if (useMegaTag2 == true)
        {
            LimelightHelpers.SetRobotOrientation(limelightName, getState( ).Pose.getRotation( ).getDegrees( ), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            if (Math.abs(getPigeon2( ).getAngularVelocityZWorld( ).getValue( ).in(DegreesPerSecond)) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            {
                doRejectUpdate = true;
            }

            if (mt2 == null || mt2.tagCount == 0)
            {
                doRejectUpdate = true;
            }
            // Reject if average tag distance is greater than 5 meters away
            else if (mt2.avgTagDist > 5)
            {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate)
            {
                fieldTypePub.set("Field2d");
                fieldPub.set(new double[ ]
                {
                        mt2.pose.getX( ), mt2.pose.getY( ), mt2.pose.getRotation( ).getDegrees( )
                });
                // setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999)); // Sample code from limelight
                // Code used by some teams to scale std devs by distance (below) and used by several teams
                setVisionMeasurementStdDevs(VecBuilder.fill(Math.pow(0.5, mt2.tagCount) * 0.75 * mt2.avgTagDist,
                        Math.pow(0.5, mt2.tagCount) * 0.75 * mt2.avgTagDist, Double.POSITIVE_INFINITY));
                addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    /**
     * Resets swerve pose and limelight orientation
     * 
     * @param pose
     *            new pose
     */
    public void resetPoseAndLimelight(Pose2d pose)
    {
        resetPose(pose);
        LimelightHelpers.SetRobotOrientation(Constants.kLLLeftName, pose.getRotation( ).getDegrees( ), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.kLLRightName, pose.getRotation( ).getDegrees( ), 0, 0, 0, 0, 0);
    }

    /**
     * Reset robot pose from dashboard widget
     */
    private void setOdometryFromDashboard( )
    {
        resetPose(new Pose2d(new Translation2d(poseXEntry.get( ), poseYEntry.get( )), new Rotation2d(poseRotEntry.get( ))));
    }

    ////////////////////////////////////////////////////////////////////////////
    /////////////////////// AUTO-ALIGN HELPERS /////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    /**
     * Initialize the arrays with the reef AprilTags
     */
    private static final int[ ] blueReefTags =
    {
            17, 18, 19, 20, 21, 22  // Must be in numerical order
    };

    private static final int[ ] redReefTags  =
    {
            8, 7, 6, 11, 10, 9      // Rotationally ordered the same as blue tags above
    };

    /**
     * Find the closest AprilTag ID to the robot and return it (always returns blue side tags only)
     * 
     * @return closestBlueTag
     *         AprilTag closest to current pose
     */
    private int findClosestReefTag(Pose2d currentPose)
    {
        if (DriverStation.getAlliance( ).orElse(Alliance.Blue) == Alliance.Red)
        {
            currentPose = FlippingUtil.flipFieldPose(currentPose);
        }

        int closestBlueTag = 0;                                                 // Variable for saving the tag with the shortest distance (0 means none found)
        double shortestDistance = Units.feetToMeters(57.0);                // field length in meters - Variable for keeping track of lowest distance (57.0 means none found)

        // Just one calculation for either tag set
        for (int i = 0; i < 6; i++)                                             // Iterate through the array of selected reef tags
        {
            Pose2d atPose = VIConsts.kATField.getTagPose(blueReefTags[i]).get( ).toPose2d( );              // Get the AT tag in Pose2d form
            double distance = currentPose.getTranslation( ).getDistance((atPose.getTranslation( )));    // Calculate the distance from the AT tag to the robotPose
            DataLogManager.log(String.format("Possible tag: %d pose: %s distance: %f", blueReefTags[i], atPose, distance));
            if (distance < shortestDistance)                                                            // If the distance is shorter than what was saved before
            {
                closestBlueTag = blueReefTags[i];                                                       // Saves cloest AT id (always in blue space)
                shortestDistance = distance;                                                            // Update new shortest distance
            }
        }

        DataLogManager.log(String.format("closest tag: %d current pose: %s - FOUND!", closestBlueTag, currentPose));
        return closestBlueTag;
    }

    /**
     * Find Goal Pose for a given blue reef AprilTag ID
     * 
     * 1) Get the closest reef AprilTag
     * 2) Retrive the a branch/face offset selection (left, middle (algae), right)
     * 3) Use the closest blue AprilTag ID and branch offset to find goal pose
     * 4) Return the goal pose (blue side only)
     * 
     * @return goalPose
     *         goal pose for the reef tag passed in
     */
    public Pose2d findGoalPose(Pose2d currentPose)
    {
        int reefTag = findClosestReefTag(currentPose);

        int reefOffset = (int) reefBranch.get( );

        int relativeReefTag = reefTag - blueReefTags[0];
        Pose2d goalPose = VIConsts.kBlueSideReefPoses[relativeReefTag][reefOffset];

        if (DriverStation.getAlliance( ).orElse(Alliance.Blue) == Alliance.Red)
        {
            goalPose = FlippingUtil.flipFieldPose(goalPose);
            reefTag = redReefTags[relativeReefTag];
        }

        DataLogManager.log(String.format("goal tag: %d goal pose %s", reefTag, goalPose));

        return goalPose;
    }

    /**
     * Create reef align command for pathfinding
     * 
     * 1) Get the goal pose (branch offset from nearest AprilTag and flip if needed
     * 2) Find a new path to the goal pose, and then follow it
     * 
     * @return reefAlignCommand
     *         command to align to a reef scoring position
     */
    public Command getAlignToReefCommand( )
    {
        Pose2d currentPose = drivePose.get( );
        Pose2d goalPose = findGoalPose(currentPose);

        return new SequentialCommandGroup(                                                                                  //
                new LogCommand("AlignToReef", String.format("ReefLevel %d ReefBranch %d current %s goal %s",  //
                        reefLevel.get( ), reefBranch.get( ), currentPose, goalPose)),                                       //
                AutoBuilder.pathfindToPose(goalPose, kPathFindConstraints, 0.0)                             //
        );
    }

    /**
     * Create reef align command for path faollowing
     * 
     * 1) Get the goal pose (branch offset from nearest AprilTag)
     * 2) Create list of waypoints
     * 3) Generate a new path
     * 4) Follow the path
     * 
     * @return reefAlignCommand
     *         command to align to a reef scoring position
     */
    public Command getAlignToReefCommand2( )
    {
        Pose2d currentPose = drivePose.get( );
        Pose2d goalPose = findGoalPose(currentPose);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, goalPose);

        PathPlannerPath path =
                new PathPlannerPath(waypoints, kPathFindConstraints, null, new GoalEndState(0.0, goalPose.getRotation( )));
        path.preventFlipping = true;

        return new SequentialCommandGroup(                                                                                    //
                new LogCommand("AlignToReef2", String.format("ReefLevel %d ReefBranch %d current %s goal %s",   //
                        reefLevel.get( ), reefBranch.get( ), currentPose, goalPose)),                                         //
                AutoBuilder.followPath(path)                                                                                  //
        );
    }

    /**
     * Create reef align command for PID driving
     * 
     * 1) Get the goal pose (branch offset from nearest AprilTag
     * 2) Get the current pose
     * 3) Generate a new path and flip if needed (use FollowPath or PathFind from PPLib)
     * 4) Follow the path
     * 
     * @return reefAlignCommand
     *         command to align to a reef scoring position
     */
    public Command getAlignToReefCommand3( )
    {
        return new SequentialCommandGroup(                                                                                                        //
                new LogCommand("AlignToReef3", String.format("ReefLevel %d ReefBranch %d", reefLevel.get( ), reefBranch.get( ))),   //
                SwervePIDController.generateCommand(this, Seconds.of(2.5))                                                              //
        );
    }

}
