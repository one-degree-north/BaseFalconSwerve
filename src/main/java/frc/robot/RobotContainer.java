package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.TunableNumber;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final String ROOT_TABLE = "Autos";

    private final TunableNumber goToPoseX = new TunableNumber(ROOT_TABLE + "/TestAutos/GoToPoseX", 14.2);
    private final TunableNumber goToPoseY = new TunableNumber(ROOT_TABLE + "/TestAutos/GoToPoseY", 1.1);
    private final TunableNumber goToPoseTheta = new TunableNumber(ROOT_TABLE + "/TestAutos/GoToPoseTheta", 180);
    private Command goToPoseTunableAuto = new GoToPoseCommand(
        new Pose2d(goToPoseX.get(), 
        goToPoseY.get(), 
        Rotation2d.fromDegrees(goToPoseTheta.get())), s_Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        /* Adding Autos */
        SmartDashboard.putData(autoChooser);
        autoChooser.addOption("GoToPoseTunableAuto", goToPoseTunableAuto);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /** Method is used to run teleop init commands without accessing Robot.java. Simply for ease of use */
    public void teleopInit() {

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Resetting command because tunable numbers may have changed
        goToPoseTunableAuto = new GoToPoseCommand(
            new Pose2d(goToPoseX.get(), 
            goToPoseY.get(), 
            Rotation2d.fromDegrees(goToPoseTheta.get())), s_Swerve);
        
        // An ExampleCommand will run in autonomous
        // return autoChooser.getSelected();
        return new PathPlannerFollowCommand("New Path", s_Swerve);
    }
}
