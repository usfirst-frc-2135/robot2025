//
// Swerve Subystem - command-based swerve subsystem
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VIConsts;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.LimelightHelpers;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem
{
  private final boolean                              m_useLimelight                  = true; // set to false when no limelight to prevent sim errors
  private static final String                        kSwerveTab                      = "Swerve";
  private static final double                        kSimLoopPeriod                  = 0.005; // 5 ms
  private Notifier                                   m_simNotifier                   = null;
  private double                                     m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d                           BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d                           RedAlliancePerspectiveRotation  = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean                                    hasAppliedOperatorPerspective   = false;
  /* Speaker AprilTag Pose for calculating distance */
  private Pose2d                                     m_allianceSpeakerATPose         = new Pose2d( );

  private final SwerveRequest.ApplyChassisSpeeds     AutoRequest                     = new SwerveRequest.ApplyChassisSpeeds( );

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization     =
      new SwerveRequest.SysIdSwerveTranslation( );
  private final SwerveRequest.SysIdSwerveRotation    RotationCharacterization        = new SwerveRequest.SysIdSwerveRotation( );
  private final SwerveRequest.SysIdSwerveSteerGains  SteerCharacterization           = new SwerveRequest.SysIdSwerveSteerGains( );

  /* Use one of these sysidroutines for your particular test */
  private SysIdRoutine                               SysIdRoutineTranslation         = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(4), null, (state) -> SignalLogger.writeString("state", state.toString( ))),
      new SysIdRoutine.Mechanism((volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this));
  @SuppressWarnings("unused")
  private final SysIdRoutine                         SysIdRoutineRotation            = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(4), null, (state) -> SignalLogger.writeString("state", state.toString( ))),
      new SysIdRoutine.Mechanism((volts) -> setControl(RotationCharacterization.withVolts(volts)), null, this));
  @SuppressWarnings("unused")
  private final SysIdRoutine                         SysIdRoutineSteer               = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(7), null, (state) -> SignalLogger.writeString("state", state.toString( ))),
      new SysIdRoutine.Mechanism((volts) -> setControl(SteerCharacterization.withVolts(volts)), null, this));

  /* Change this to the sysid routine you want to test */
  private final SysIdRoutine                         RoutineToApply                  = SysIdRoutineTranslation;

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance                 inst                            = NetworkTableInstance.getDefault( );

  /* Robot pose for field positioning */
  private final NetworkTable                         table                           = inst.getTable("Pose");
  private final DoubleArrayPublisher                 fieldPub                        =
      table.getDoubleArrayTopic("llPose").publish( );
  private final StringPublisher                      fieldTypePub                    = table.getStringTopic(".type").publish( );

  /* Robot pathToPose constraints */
  private final PathConstraints                      kPathFindConstraints            = new PathConstraints( // 
      2.5,       // kMaxVelocityMps                               (slowed from 3.0 for testing)    
      2.5, // kMaxAccelerationMpsSq                         (slowed from 3.0 for testing)  
      1.5 * Math.PI,            // kMaxAngularSpeedRadiansPerSecond              (slowed from 2.0 * Math.PI for testing)  
      1.5 * Math.PI             // kMaxAngularSpeedRadiansPerSecondSquared       (slowed from 1.5 * Math.PIfor testing)  
  );

  // Shuffleboard objects
  ShuffleboardTab                                    swerveTab                       = Shuffleboard.getTab(kSwerveTab);
  ShuffleboardLayout                                 poseList                        =
      swerveTab.getLayout("Pose", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 3);

  GenericEntry                                       poseXEntry                      =
      poseList.add("X", 0.0).withPosition(0, 0).getEntry( );
  GenericEntry                                       poseYEntry                      =
      poseList.add("Y", 0.0).withPosition(0, 1).getEntry( );
  GenericEntry                                       poseRotEntry                    =
      poseList.add("rotation", 0.0).withPosition(0, 2).getEntry( );
  ComplexWidget                                      setPose                         =
      poseList.add("SetPose", new InstantCommand(( ) -> setOdometryFromDashboard( )).ignoringDisable(true)).withPosition(0, 2);
  GenericEntry                                       shooterDistanceEntry            =
      swerveTab.add("shooterDistance", 0.0).withPosition(8, 2).withSize(2, 1).getEntry( );

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
      SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    configurePathPlanner( );
    if (Utils.isSimulation( ))
    {
      startSimThread( );
    }
  }

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, modules);
    configurePathPlanner( );
    if (Utils.isSimulation( ))
    {
      startSimThread( );
    }
  }

  private void configurePathPlanner( )
  {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations)
    {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm( ));
    }

    AutoBuilder.configureHolonomic(( ) -> this.getState( ).Pose,      // Supplier of current robot pose
        this::seedFieldRelative,                                      // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,                           // Supplier of chassis speeds
        (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),  // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(new PIDConstants(25, 0, 0), new PIDConstants(10, 0, 0), TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius, new ReplanningConfig( )),                // Path following config
        ( ) -> DriverStation.getAlliance( ).orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be flipped for Red vs Blue (normally case)
        this);                                                        // Subsystem for requirements
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
  {
    return run(( ) -> this.setControl(requestSupplier.get( )));
  }

  public Command getAutoPathCommand(String autoName)
  {
    return new PathPlannerAuto(autoName).withName("swervePPAuto");
  }

  public Command getPathCommand(PathPlannerPath ppPath)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(ppPath).withName("swervePPPath");
  }

  /*
   * Both the sysid commands are specific to one particular sysid routine, change
   * which one you're trying to characterize
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
  {
    return RoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction)
  {
    return RoutineToApply.dynamic(direction);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds( )
  {
    return m_kinematics.toChassisSpeeds(getState( ).ModuleStates);
  }

  private void startSimThread( )
  {
    m_lastSimTime = Utils.getCurrentTimeSeconds( );

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(( ) ->
    {
      final double currentTime = Utils.getCurrentTimeSeconds( );
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage( ));
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /*
   * Limelight MegaTag example code for vision processing
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
      LimelightHelpers.SetRobotOrientation("limelight", m_odometry.getEstimatedPosition( ).getRotation( ).getDegrees( ), 0, 0, 0,
          0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

      if (Math.abs(m_pigeon2.getRate( )) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if (mt2 == null || mt2.tagCount == 0)
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
        setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      }
    }
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    /* Periodically try to apply the operator perspective */
    /*
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS
     * state
     */
    /* This allows us to correct the perspective in case the robot code restarts mid-match */
    /* Otherwise, only check and apply the operator perspective if the DS is disabled */
    /*
     * This ensures driving behavior doesn't change until an explicit disable event occurs during
     * testing
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled( ))
    {
      DriverStation.getAlliance( ).ifPresent((allianceColor) ->
      {
        this.setOperatorPerspectiveForward(
            allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation);
        m_allianceSpeakerATPose = VIConsts.kAprilTagPoses.get((allianceColor == Alliance.Red) ? 4 : 7);
        hasAppliedOperatorPerspective = true;
      });
    }

    double speakerTagDistance = this.getState( ).Pose.getTranslation( ).getDistance(m_allianceSpeakerATPose.getTranslation( ));
    shooterDistanceEntry.setDouble(Units.metersToInches(speakerTagDistance));

    if (m_useLimelight && Robot.isReal( ))
    {
      visionUpdate( );
    }
  }

  public Command drivePathtoPose(CommandSwerveDrivetrain drivetrain, Pose2d pose)
  {
    DataLogManager.log(String.format("drivePathToPose: Alliance %s target pose %s", DriverStation.getAlliance( ), pose));

    return AutoBuilder.pathfindToPoseFlipped(pose, kPathFindConstraints, 0.0);
  }

  private void setOdometryFromDashboard( )
  {
    seedFieldRelative(        //
        new Pose2d(           //
            new Translation2d(poseXEntry.getDouble(0.0), poseYEntry.getDouble(0.0)), //
            new Rotation2d(poseRotEntry.getDouble(0.0)))        //
    );
  }

}
