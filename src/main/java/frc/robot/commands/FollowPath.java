package frc.robot.commands;

import static frc.robot.Constants.Swerve.*;

import java.lang.reflect.Field;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  PathPlannerTrajectory trajectory;
  private boolean initialPath;
  private Field2d trajectoryVisual;

  

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


  public FollowPath(PathPlannerTrajectory trajectory, Swerve subsystem, boolean initialPath) {
    super(
        trajectory,
        subsystem::getPhotonPose,
        swerveKinematics,
        subsystem.getAutoXController(),
        subsystem.getAutoYController(),
        subsystem.getAutoThetaController(),
        subsystem::setModuleStates,
        true,
        subsystem);

    this.drivetrain = subsystem;
    this.trajectory = trajectory;
    this.initialPath = initialPath;
    this.trajectoryVisual = new Field2d();

    addRequirements(drivetrain);
  }

  public void changeTrajectory(PathPlannerTrajectory traj) {
    super.trajectory = traj;
    this.trajectory = traj;
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
    // TODO: check trajectory.fromGUI in the super class

    if (initialPath) {
      // reset odometry to the starting pose of the trajectory
      this.drivetrain.resetPhotonPose(this.trajectory.getInitialState().poseMeters);
    }
    
    if (trajectory != null){
      trajectoryVisual.getObject("Auto").setTrajectory(trajectory);
      SmartDashboard.putData("Auto Trajectory", trajectoryVisual);

    }    

    // reset the theta controller such that old accumulated ID values aren't used with the new path
    //      this doesn't matter if only the P value is non-zero, which is the current behavior
    drivetrain.getAutoXController().reset();
    drivetrain.getAutoYController().reset();
    drivetrain.getAutoThetaController().reset();

    drivetrain.getAutoXController().setTolerance(0.02);
    drivetrain.getAutoYController().setTolerance(0.02);
    drivetrain.getAutoThetaController().setTolerance(0.02);

    super.initialize();
  }

  @Override
  public void execute() {
    trajectoryVisual.setRobotPose(this.drivetrain.getPhotonPose());
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
    super.end(interrupted);
    this.drivetrain.drive(new Translation2d(0, 0), 0, false, false);
  }
}
