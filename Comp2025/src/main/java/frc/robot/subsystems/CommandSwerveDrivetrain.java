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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
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
    private final Field2d               field               = new Field2d();
    private final FieldObject2d         llPoseLeft          = field.getObject("llPose-left"); 
    private final FieldObject2d         llPoseRight         = field.getObject("llPose-right"); 

    private final NetworkTable              driveStateTable = ntInst.getTable("DriveState");
    private final StructSubscriber<Pose2d>  driveStatePose  = driveStateTable.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d( ));

    /* Robot set pose */
    private final NetworkTable          swerveTable         = ntInst.getTable("swerve");
    private final DoubleArrayPublisher  setPosePub          = swerveTable.getDoubleArrayTopic("setPose").publish();
    private final DoubleArraySubscriber setPoseSub          = swerveTable.getDoubleArrayTopic("setPose").subscribe(new double[3]);

    private MedianFilter leftFilter = new MedianFilter(5);
    private MedianFilter rightFilter = new MedianFilter(5);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("swerve");

    private BooleanPublisher leftUpdate = table.getBooleanTopic("LeftUpdate").publish();
    private BooleanPublisher rightUpdate = table.getBooleanTopic("RightUpdate").publish();

    private double [] moduleDistances = {0, 0, 0, 0};

    /* Robot pathToPose constraints */
    private final PathConstraints       kPathFindConstraints = new PathConstraints( // 
        3.5,            // kMaxVelocityMps
        3.5,      // kMaxAccelerationMpsSq
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

        if (m_useLimelight) {
            double left = (visionUpdate(Constants.kLLLeftName, llPoseLeft)) ? 1.0 : 0.0;
            leftUpdate.set(leftFilter.calculate(left) > 0.5);

            double right = (visionUpdate(Constants.kLLRightName, llPoseRight)) ? 1.0 : 0.0;
            rightUpdate.set(rightFilter.calculate(right) > 0.5);
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
        setPosePub.set(new double[3]);
        SmartDashboard.putData("Field", field);

        // Get the default instance of NetworkTables that was created automatically when the robot program starts
        SmartDashboard.putData("SetPose", getResetPoseCommand( ));

        SmartDashboard.putData("AlignToReefPID", getAlignToReefPIDCommand( ));
        SmartDashboard.putData("AlignToReefFollow", new DeferredCommand(( ) -> getAlignToReefFollowCommand( ), Set.of(this)));
        SmartDashboard.putData("AlignToReefPPFind", new DeferredCommand(( ) -> getAlignToReefPPFindCommand( ), Set.of(this)));
    }

    /****************************************************************************
     * 
     * Limelight MegaTag example code for vision processing
     *
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to goal
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact
     * implementation of how to use vision should be tuned per-robot and to the team's specification.
     */
    private boolean visionUpdate(String limelightName, FieldObject2d fieldObject)
    {
        boolean useMegaTag2 = true; // set to false to use MegaTag1
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
                    if (mt1.rawFiducials[0].ambiguity > 0.7)
                    {
                        doRejectUpdate = true;
                    }
                    if (mt1.rawFiducials[0].distToCamera > 3.0)
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
                fieldObject.setPose(mt1.pose.getX( ), mt1.pose.getY( ), mt1.pose.getRotation( ));

                setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
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
            else if (mt2.avgTagDist > 5.0)
            {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate)
            {
                final double kBase = 0.5;
                final double kProportional = 0.9;
                fieldObject.setPose(mt2.pose.getX( ), mt2.pose.getY( ), mt2.pose.getRotation( ));

                // Code used by some teams to scale std devs by distance (below) and used by several teams
                setVisionMeasurementStdDevs(VecBuilder.fill(    //
                        Math.pow(kBase, mt2.tagCount) * kProportional * mt2.avgTagDist, //
                        Math.pow(kBase, mt2.tagCount) * kProportional * mt2.avgTagDist, //
                        Double.POSITIVE_INFINITY));
                addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }

        return !doRejectUpdate;
    }

    /****************************************************************************
     * 
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

    ////////////////////////////////////////////////////////////////////////////
    /////////////////////// COMMAND FACTORIES //////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    /****************************************************************************
     *
     * Reset robot pose from dashboard widget
     */
    private Command getResetPoseCommand( )
    {
        return this
                .runOnce(( ) -> resetPoseAndLimelight(new Pose2d(new Translation2d(setPoseSub.get( )[0], setPoseSub.get( )[1]),
                        new Rotation2d(setPoseSub.get( )[2])))) //
                .withName("ResetOdometry").ignoringDisable(true);
    }

    /****************************************************************************
     *
     * Log current module positions and calculate an average distance if requested
     */
    public Command getModulePositionsCommand(boolean end)
    {
        return this.runOnce(( ) ->
        {
            if (!end)
            {
                for (int i = 0; i < 4; i++)
                {
                    moduleDistances[i] = this.getState( ).ModulePositions[i].distanceMeters;
                }
            }
            else
            {
                double average = 0;
                for (int i = 0; i < 4; i++)
                {
                    moduleDistances[i] = this.getState( ).ModulePositions[i].distanceMeters - moduleDistances[i];
                    average += Math.abs(moduleDistances[i]);
                }
                average /= 4;
                DataLogManager.log(String.format("%s:  0: %.3f 1: %.3f 2: %.3f 3: %.3f average %.3f", this.getName( ),
                        moduleDistances[0], moduleDistances[1], moduleDistances[2], moduleDistances[3], average));
            }
        }).ignoringDisable(true);
    }

    /****************************************************************************
     *
     * Construct a path following command used in autonomous
     */
    public Command getPathCommand(PathPlannerPath ppPath)
    {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(ppPath).withName("SwerveFollowPath");
    }

    /****************************************************************************
     *
     * Create reef align command for PID driving
     * 
     * @return reefAlignCommand
     *         command to align to a reef scoring position
     */
    public Command getAlignToReefPIDCommand( )
    {
        return SwervePIDController.generateCommand(this, Seconds.of(2.5)).withName("AlignToReefPID");
    }

    /****************************************************************************
     *
     * Create reef align command for path following
     * 
     * 1) Get the current pose
     * 2) Find the goal pose (branch offset from nearest AprilTag)
     * 2) Create list of waypoints
     * 3) Generate a new path
     * 4) Follow the path
     * 
     * @return reefAlignCommand
     *         command to align to a reef scoring position
     */
    private Command getAlignToReefFollowCommand( )
    {
        Pose2d currentPose = driveStatePose.get( );
        Pose2d goalPose = Vision.findGoalPose(currentPose);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, goalPose);

        PathPlannerPath path =
                new PathPlannerPath(waypoints, kPathFindConstraints, null, new GoalEndState(0.0, goalPose.getRotation( )));
        path.preventFlipping = true;

        return new SequentialCommandGroup(                                                                                     //
                new LogCommand("AlignReefFollow", String.format("current %s goal %s", currentPose, goalPose)),   //
                AutoBuilder.followPath(path)                                                                                   //
        ).withName("AlignToReefFollow");
    }

    /****************************************************************************
     *
     * Create reef align command for pathfinding
     * 
     * 1) Get the current pose
     * 2) Find the goal pose (branch offset from nearest AprilTag)
     * 2) Find a new path to the goal pose, and then follow it
     * 
     * @return reefAlignCommand
     *         command to align to a reef scoring position
     */
    private Command getAlignToReefPPFindCommand( )
    {
        Pose2d currentPose = driveStatePose.get( );
        Pose2d goalPose = Vision.findGoalPose(currentPose);

        return new SequentialCommandGroup(                                                                                      //
                new LogCommand("AlignReefPPFind", String.format("current %s goal %s", currentPose, goalPose)),    //
                AutoBuilder.pathfindToPose(goalPose, kPathFindConstraints, 0.0)                                 //
        ).withName("AlignToReefPPFind");
    }
}
