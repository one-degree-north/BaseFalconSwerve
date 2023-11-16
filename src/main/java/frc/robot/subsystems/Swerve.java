package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private AHRS gyro;
    private PoseEstimatorSubsystem PoseEstimator;
    private Field2d field2d;
    private ChassisSpeeds chassisSpeeds;

    public Swerve() {
        gyro = new AHRS(SerialPort.Port.kMXP);
        gyro.calibrate();
        gyro.reset();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        Supplier<Rotation2d> rotSupplier = () -> getYaw();
        Supplier<SwerveModulePosition[]> modSupplier = () -> getModulePositions();

        PoseEstimator = new PoseEstimatorSubsystem(rotSupplier, modSupplier);
        PoseEstimator.setAlliance(DriverStation.getAlliance());

        field2d = new Field2d();
        SmartDashboard.putData(field2d);

        chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw()
        )
        : new ChassisSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation);
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public PathPlannerTrajectory generateOnTheFlyTrajectory(Pose2d targetPose) {
        return PathPlanner.generatePath(
            new PathConstraints(
                Constants.Swerve.maxSpeed,
                Constants.Swerve.maxAngularVelocity),
            PathPoint.fromCurrentHolonomicState(Constants.Swerve.useVision ? this.getPhotonPose() : this.getOdometryPose(), this.getCurrentChassisSpeeds()),
            new PathPoint(
                targetPose.getTranslation(), Rotation2d.fromDegrees(0), targetPose.getRotation()));
    }

    public PathPlannerTrajectory generateOnTheFlyTrajectory(
        Pose2d targetPose, double driveVelocityConstraint, double driveAccelConstraint) {
        var path =
            PathPlanner.generatePath(
                new PathConstraints(driveVelocityConstraint, driveAccelConstraint),
                PathPoint.fromCurrentHolonomicState(Constants.Swerve.useVision ? this.getPhotonPose() : this.getOdometryPose(), this.getCurrentChassisSpeeds()),
                new PathPoint(
                    targetPose.getTranslation(), Rotation2d.fromDegrees(0), targetPose.getRotation()));
    
        return path;
    }
  
    public PathPlannerTrajectory generateOnTheFlyTrajectory(
        List<Pose2d> targetPoses, double driveVelocityConstraint, double driveAccelConstraint) {
  
        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
    
        points.add(PathPoint.fromCurrentHolonomicState(Constants.Swerve.useVision ? this.getPhotonPose() : this.getOdometryPose(), chassisSpeeds));
    
        for (Pose2d pos : targetPoses) {
            points.add(new PathPoint(pos.getTranslation(), pos.getRotation()));
        }
    
        var path =
            PathPlanner.generatePath(
                new PathConstraints(driveVelocityConstraint, driveAccelConstraint), points);
    
        return path;
    }

    public Pose2d getOdometryPose() {
        return swerveOdometry.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public Pose2d getPhotonPose() {
        return PoseEstimator.getCurrentPose();
    }

    public void resetPhotonPose(Pose2d pose) {
        PoseEstimator.setCurrentPose(pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return chassisSpeeds;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return gyro.getRotation2d();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        field2d.setRobotPose(getPhotonPose());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }

        SmartDashboard.putNumber("Pose Estimator X (m)", getPhotonPose().getX());
        SmartDashboard.putNumber("Pose Estimator Y (m)", getPhotonPose().getY());
        SmartDashboard.putNumber("Pose Estimator Theta (deg)", getPhotonPose().getRotation().getDegrees());
    }
}