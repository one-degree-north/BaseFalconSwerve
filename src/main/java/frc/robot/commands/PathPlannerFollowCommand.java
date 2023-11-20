// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PathPlannerFollowCommand extends FollowPath {
  private final static String ROOT_TABLE = "PathPlanner_Follow";

  private final static TunableNumber tunableVelocityConstraint = 
  new TunableNumber(ROOT_TABLE + "/velocityConstraint", Constants.AutoConstants.velocityConstraint);
  private final static TunableNumber tunableAccelerationConstraint = 
  new TunableNumber(ROOT_TABLE + "/accelerationConstraint", Constants.AutoConstants.accelerationConstraint);

  public PathPlannerFollowCommand(String pathName, Swerve drivetrain) {
    super(PathPlanner.loadPath(pathName, 
      new PathConstraints(tunableVelocityConstraint.get(), tunableAccelerationConstraint.get())), 
    drivetrain, true);
  }

}
