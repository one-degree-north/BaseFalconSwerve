package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.util.TunableNumber;
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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private AHRS gyro;
    private PoseEstimatorSubsystem PoseEstimator;
    private ChassisSpeeds chassisSpeeds;
    private Alliance pastAlliance;

    private final static String ROOT_TABLE = "Swerve";

    private final static TunableNumber tunableX_kP = new TunableNumber(ROOT_TABLE + "/X_kP", Constants.AutoConstants.X_kP);
    private final static TunableNumber tunableY_kP = new TunableNumber(ROOT_TABLE + "/Y_kP", Constants.AutoConstants.Y_kP);
    private final static TunableNumber tunableTHETA_kP = new TunableNumber(ROOT_TABLE + "/THETA_kP", Constants.AutoConstants.THETA_kP);

    public final PIDController autoXController = new PIDController(tunableX_kP.get(), 0, 0);
    public final PIDController autoYController = new PIDController(tunableY_kP.get(), 0, 0);
    public final PIDController autoThetaController = new PIDController(tunableTHETA_kP.get(), 0, 0);

    public Swerve() {
        pastAlliance = DriverStation.getAlliance();

        gyro = new AHRS(SerialPort.Port.kMXP);

        // Calibrate gyro, and reset after calibration
        gyro.calibrate();
        int counter = 0; boolean timedOut = false;
        while (gyro.isCalibrating()){
            if (counter > 4) {
                timedOut = true;
                break;
            }
            Timer.delay(0.5);
            counter++;
        }
        if (!timedOut) {
            System.out.println("Gyro calibration done!");
            gyro.reset();
        }
        else {
            System.out.println("Gyro calibration timed out!");
        }

        // Zero gyro after reset (shouldn't technically be needed)
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

        Supplier<Rotation2d> rotSupplier = () -> getYaw();
        Supplier<SwerveModulePosition[]> modSupplier = () -> getModulePositions();

        PoseEstimator = new PoseEstimatorSubsystem(rotSupplier, modSupplier);
        PoseEstimator.setAlliance(DriverStation.getAlliance());

        chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        autoThetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw()
        )
        : new ChassisSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation);
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(this.chassisSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public PIDController getAutoXController() {
        return autoXController;
    }

    public PIDController getAutoYController() {
        return autoYController;
    }

    public PIDController getAutoThetaController() {
        return autoThetaController;
    }
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }

    }

    public Rotation2d getHeading(Pose2d initial, Pose2d end) {
        double distanceX = end.getX()-initial.getX();
        double distanceY = end.getY()-initial.getY();
        return Rotation2d.fromRadians(
            Math.atan(distanceY/distanceX));
    }

    public PathPlannerTrajectory generateOnTheFlyTrajectory(Pose2d targetPose) {
        return PathPlanner.generatePath(
            new PathConstraints(
                Constants.Swerve.maxSpeed,
                Constants.Swerve.maxAngularVelocity),
                PathPoint.fromCurrentHolonomicState(this.getPhotonPose(), this.getCurrentChassisSpeeds()),
            new PathPoint(
                targetPose.getTranslation(), Rotation2d.fromDegrees(0), targetPose.getRotation()));
    }

    // only testing on this generateonthefly method
    public PathPlannerTrajectory generateOnTheFlyTrajectory(
        Pose2d targetPose, double driveVelocityConstraint, double driveAccelConstraint) {

        var path =
            PathPlanner.generatePath(
                new PathConstraints(driveVelocityConstraint, driveAccelConstraint),
                PathPoint.fromCurrentHolonomicState(this.getPhotonPose(), this.getCurrentChassisSpeeds()),
                new PathPoint(
                    targetPose.getTranslation(), this.getHeading(this.getPhotonPose(), targetPose), targetPose.getRotation()));
    
        return path;
    }
  
    public PathPlannerTrajectory generateOnTheFlyTrajectory(
        List<Pose2d> targetPoses, double driveVelocityConstraint, double driveAccelConstraint) {
  
        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
    
        points.add(PathPoint.fromCurrentHolonomicState(this.getPhotonPose(), this.getCurrentChassisSpeeds()));
    
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
        return this.chassisSpeeds;
    }

    //TODO: make sure everything with gyro works as intended
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
        this.chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());

        if (tunableX_kP.hasChanged()) {autoXController.setPID(tunableX_kP.get(), 0, 0);}
        if (tunableY_kP.hasChanged()) {autoYController.setPID(tunableY_kP.get(), 0, 0);}
        if (tunableTHETA_kP.hasChanged()) {autoThetaController.setPID(tunableTHETA_kP.get(), 0, 0);}

        // Set alliance for Pose Estimator if it is changed.
        if (DriverStation.getAlliance() == pastAlliance) 
            PoseEstimator.setAlliance(DriverStation.getAlliance());
        pastAlliance = DriverStation.getAlliance();

        SmartDashboard.putNumber("Gyro Rotation", getYaw().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }
    }
}