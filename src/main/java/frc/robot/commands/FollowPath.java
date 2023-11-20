package frc.robot.commands;

import static frc.robot.Constants.Swerve.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

//TODO: Test paths on both blue/red alliance

/**
 * This command, when executed, instructs the drivetrain subsystem to follow the specified
 * trajectory, presumably during the autonomous period. The superclass' execute method invokes the
 * drivetrain subsystem's setSwerveModuleStates method to follow the trajectory.
 *
 * <p>Requires: the Drivetrain subsystem (handled by superclass)
 *
 * <p>Finished When: the time of the specified path has elapsed (handled by superclass)
 *
 * <p>At End: stops the drivetrain
 */
public class FollowPath extends PPSwerveControllerCommand {
  private Swerve drivetrain;
  private PathPlannerTrajectory trajectory;
  private boolean initialPath;
  private final static String ROOT_TABLE = "Follow_Path";

  private final static TunableNumber tunableX_kP = new TunableNumber(ROOT_TABLE + "/X_kP", Constants.AutoConstants.X_kP);
  private final static TunableNumber tunableY_kP = new TunableNumber(ROOT_TABLE + "/Y_kP", Constants.AutoConstants.Y_kP);
  private final static TunableNumber tunableTHETA_kP = new TunableNumber(ROOT_TABLE + "/THETA_kP", Constants.AutoConstants.THETA_kP);

  private final static PIDController autoXController = new PIDController(tunableX_kP.get(), 0, 0);
  private final static PIDController autoYController = new PIDController(tunableY_kP.get(), 0, 0);
  private final static PIDController autoThetaController = new PIDController(tunableTHETA_kP.get(), 0, 0);

  /**
   * Constructs a new FollowPath object.
   *
   * <p>CAVEAT: This helper does not support useAllianceColor.
   *
   * @param trajectory the specified trajectory created by PathPlanner
   * @param subsystem the drivetrain subsystem required by this command
   * @param initialPath true, if this trajectory is the first in a sequence of trajectories or the
   *     only trajectory, in which case the gyro and odometry will be initialized to match the start
   *     of trajectory; false, if this trajectory is a subsequent trajectory in which case the gyro
   *     and odometry will not be re-initialized in order to ensure a smooth transition between
   *     trajectories
   */

   // TODO: Check if useAllianceColor should be true (probably not)

  public FollowPath(PathPlannerTrajectory trajectory, Swerve subsystem, boolean initialPath) {
    super(
        trajectory,
        subsystem::getPhotonPose,
        swerveKinematics,
        autoXController,
        autoYController,
        autoThetaController,
        subsystem::setModuleStates,
        false,
        subsystem);

    this.drivetrain = subsystem;
    this.trajectory = trajectory;
    this.initialPath = initialPath;
  }

  /**
   * This method is invoked once when this command is scheduled. If the trajectory is the first in a
   * sequence of trajectories or the only trajectory, initialize the gyro and odometry to match the
   * start of trajectory. PathPlanner sets the origin of the field to the lower left corner (i.e.,
   * the corner of the field to the driver's right). Zero degrees is away from the driver and
   * increases in the CCW direction. It is critical that this initialization occurs in this method
   * and not the constructor as this object is constructed well before the command is scheduled.
   */
  @Override
  public void initialize() {
    super.initialize();

    if (initialPath) {
      // reset odometry to the starting pose of the trajectory
      this.drivetrain.resetPhotonPose(this.trajectory.getInitialState().poseMeters);
    }

    // reset the theta controller such that old accumulated ID values aren't used with the new path
    //      this doesn't matter if only the P value is non-zero, which is the current behavior
    autoXController.reset();
    autoYController.reset();
    autoThetaController.reset();

  }

  @Override
  public void execute() {
    if (tunableX_kP.hasChanged()) {autoXController.setPID(tunableX_kP.get(), 0, 0);}
    if (tunableY_kP.hasChanged()) {autoYController.setPID(tunableY_kP.get(), 0, 0);}
    if (tunableTHETA_kP.hasChanged()) {autoThetaController.setPID(tunableTHETA_kP.get(), 0, 0);}

    super.execute();
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.drive(new Translation2d(0, 0), 0, true, true);
    super.end(interrupted);
  }
}
