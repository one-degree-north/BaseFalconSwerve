// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class GoToPoseCommand extends FollowPath {
  private final static String ROOT_TABLE = "Go_To_Pose";

  private final static TunableNumber tunableVelocityConstraint = 
  new TunableNumber(ROOT_TABLE + "/velocityConstraint", Constants.AutoConstants.velocityConstraint);
  private final static TunableNumber tunableAccelerationConstraint = 
  new TunableNumber(ROOT_TABLE + "/accelerationConstraint", Constants.AutoConstants.accelerationConstraint);

  public GoToPoseCommand(Pose2d targetPose, Swerve drivetrain) {
    super(drivetrain.generateOnTheFlyTrajectory(targetPose, 
    tunableVelocityConstraint.get(), tunableAccelerationConstraint.get()), 
    drivetrain, false);
  }

}
