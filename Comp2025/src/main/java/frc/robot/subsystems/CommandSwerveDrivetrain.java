//
// Swerve Subystem - command-based swerve subsystem
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.VIConsts;
import frc.robot.Robot;
import frc.robot.commands.LogCommand;
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
    private final NetworkTableInstance  inst                     = NetworkTableInstance.getDefault( );

    /* Robot pose for field positioning */
    private final NetworkTable          table                    = inst.getTable("Pose");
    private final DoubleArrayPublisher  fieldPub                 = table.getDoubleArrayTopic("llPose").publish( );
    private final StringPublisher       fieldTypePub             = table.getStringTopic(".type").publish( );

    // Network tables publisher objects
    DoubleEntry                         poseXEntry;
    DoubleEntry                         poseYEntry;
    DoubleEntry                         poseRotEntry;

    /* Robot pathToPose constraints */
    private final PathConstraints       kPathFindConstraints     = new PathConstraints( // 
        2.5,       // kMaxVelocityMps                               (slowed from 3.0 for testing)    
        2.5, // kMaxAccelerationMpsSq                         (slowed from 3.0 for testing)  
        1.5 * Math.PI,            // kMaxAngularSpeedRadiansPerSecond              (slowed from 2.0 * Math.PI for testing)  
        1.5 * Math.PI             // kMaxAngularSpeedRadiansPerSecondSquared       (slowed from 1.5 * Math.PIfor testing)  
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
            visionUpdate( );
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

    private void initDashboard( )
    {
        // Get the default instance of NetworkTables that was created automatically when the robot program starts
        NetworkTable table = inst.getTable("swerve");

        poseXEntry = table.getDoubleTopic("X").getEntry(0.0);
        poseYEntry = table.getDoubleTopic("Y").getEntry(0.0);
        poseRotEntry = table.getDoubleTopic("rotation").getEntry(0.0);
        SmartDashboard.putData("SetPose", new InstantCommand(( ) -> setOdometryFromDashboard( )).ignoringDisable(true));
        SmartDashboard.putData("FaceSelector", new InstantCommand(( ) -> findClosestReefTag( )).ignoringDisable(true));
        SmartDashboard.putData("DrivePathToPoseCommand", getDrivePathToPoseCommand(this, findTargetPose( )));
    }

    public Command getPathCommand(PathPlannerPath ppPath)
    {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(ppPath).withName("swervePPPath");
    }

    /*
     * Limelight MegaTag example code for vision processing
     */
    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact
     * implementation of how to use vision should be tuned per-robot and to the team's specification.
     */
    private void visionUpdate( )
    {
        boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if (useMegaTag2 == false)
        {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

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

            if (!doRejectUpdate)
            {
                setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        }
        else if (useMegaTag2 == true)
        {
            LimelightHelpers.SetRobotOrientation("limelight", getState( ).Pose.getRotation( ).getDegrees( ), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

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
                setVisionMeasurementStdDevs(VecBuilder.fill(Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist,
                        Math.pow(0.5, mt2.tagCount) * 2 * mt2.avgTagDist, Double.POSITIVE_INFINITY));
                addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    public Command getDrivePathToPoseCommand(CommandSwerveDrivetrain drivetrain, Pose2d pose)
    {
        DataLogManager.log(String.format("drivePathToPose: Alliance %s target pose %s", DriverStation.getAlliance( ), pose));
        return AutoBuilder.pathfindToPoseFlipped(pose, kPathFindConstraints, 0.0);
    }

    public Command getReefAlignmentCommand( )
    {
        Pose2d targetPose = findTargetPose( );
        NetworkTable table = inst.getTable(Constants.kRobotString);

        // TODO: Updates needed
        //  1) The path following command will need to have a Path created from the current robot pose and the desired pose
        //  2) So we need to get the reef offset from the publisher created in robotContainer, and also get the closest AT tag face
        //  3) Create a pose that will let us score
        //  4) Generate a path that starts with the current pose and ends at the target pose
        //  6) Options:
        //      a) PPLib FollowPath would follow this directly
        //      b) PPLib PathFindToPose should also get to the correct destination, but may take longer
        //      c) PPLib PathFindToPath is probably the highest accuracy, but may take more work

        // Note that getReefAlignment can do all the work before returning the PPLib call we need to run the path

        return new SequentialCommandGroup(                                                                              //
                AutoBuilder.pathfindToPoseFlipped(targetPose, kPathFindConstraints, 0.0),               //
                new LogCommand("Desired Offset", String.format("Desired Offset .......................",
                        table.getIntegerTopic(VIConsts.kReefOffsetString).subscribe(0).get( )))              //
        );
    }

    /*
     * Initialize the array with the reef tags and faces
     */
    private int[ ] blueReefTags =
    {
            17, 18, 19, 20, 21, 22
    };

    private int[ ] redReefTags  =
    {
            8, 7, 6, 11, 10, 9
    };

    /*
     * The calculation to find the closest face tag ID to the robot and return the tag ID
     */
    public int findClosestReefTag( )
    {
        Alliance alliance = DriverStation.getAlliance( ).orElse(Alliance.Blue); // This will always return either Red or Blue and removes the optional type
                                                                               // The orElse means "use Blue if no alliance is found"
        int tagsToUse[];                                                        // The array to be used in the calculation

        tagsToUse = (alliance.equals(Alliance.Blue)) ? blueReefTags : redReefTags; // Select the red or blue tag array

        Pose2d robotPose = getState( ).Pose;                                    // Get the robot pose (2d) in a convenient local variable
        int closestTag = 0;                                                     // Variable for saving the tag with the shortest distance (0 means none found)
        double shortestDistance = Units.feetToMeters(57.0);                 // field length in meters - Variable for keeping track of lowest distance (54.0 means none found)

        // Just one calculation for either tag set
        for (int i = 0; i < 6; i++)                                             // Iterate through the array of selected reef tags
        {
            Pose2d atPose = VIConsts.kATField.getTagPose(tagsToUse[i]).get( ).toPose2d( );          // Get the AT tag in Pose2d form
            double distance = robotPose.getTranslation( ).getDistance((atPose.getTranslation( )));  // Calculate the distance from the AT tag to the robotPose
            DataLogManager.log(String.format("tag: %d pose: %s robot: %f", tagsToUse[i], atPose.getTranslation( ), distance));
            if (distance < shortestDistance)                                                        // If the distance is shorter than what was saved before
            {
                closestTag = blueReefTags[i];                                                       // Saves cloest AT id (always in blue space)
                shortestDistance = distance;                                                        // Update new shortest distance
            }
        }

        DataLogManager.log(String.format("closest AT tag is %d", closestTag));
        return closestTag;
    }

    /*
     * We want a method that:
     * - given a branch/face selection (left, middle, right)
     * - finds the closest face to the robot (closest AT tag)
     * - selects either branch left pose or right pose, or selects the algae pose
     * - returns the target pose
     * 
     */
    public Pose2d findTargetPose( )
    {
        int desiredReefTag = findClosestReefTag( );
        Pose2d desiredPose2d = new Pose2d( );

        NetworkTable table = inst.getTable(Constants.kRobotString);
        int scoringOffset = (int) table.getIntegerTopic(VIConsts.kReefOffsetString).subscribe(0).get( );

        switch (desiredReefTag)
        {
            case 17 :
            {
                if (scoringOffset == 0)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[0][0];
                }
                else if (scoringOffset == 1)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[0][2];
                }
                else
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[0][1];
                }
                break;
            }

            case 18 :
            {
                if (scoringOffset == 0)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[1][0];
                }
                else if (scoringOffset == 1)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[1][2];
                }
                else
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[1][1];
                }
                break;
            }
            case 19 :
            {
                if (scoringOffset == 0)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[2][0];
                }
                else if (scoringOffset == 1)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[2][2];
                }
                else
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[2][1];
                }
                break;
            }
            case 20 :
            {
                if (scoringOffset == 0)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[3][0];
                }
                else if (scoringOffset == 1)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[3][2];
                }
                else
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[3][1];
                }
                break;
            }
            case 21 :
            {
                if (scoringOffset == 0)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[4][0];
                }
                else if (scoringOffset == 1)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[4][2];
                }
                else
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[4][1];
                }
                break;
            }
            case 22 :
            {
                if (scoringOffset == 0)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[5][0];
                }
                else if (scoringOffset == 1)
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[5][2];
                }
                else
                {
                    desiredPose2d = VIConsts.kBlueSideReefPoses[5][1];
                }
                break;
            }

        }

        DataLogManager.log(String.format("closest AT tag: %d desiredPose %s", desiredReefTag, desiredPose2d));
        return desiredPose2d;
    }

    private void setOdometryFromDashboard( )
    {
        resetPose(        //
                new Pose2d(           //
                        new Translation2d(poseXEntry.get(0.0), poseYEntry.get(0.0)), //
                        new Rotation2d(poseRotEntry.get(0.0)))        //
        );
    }

}
