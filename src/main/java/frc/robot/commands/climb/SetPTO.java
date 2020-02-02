/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class SetPTO extends InstantCommand {
  private boolean climbing;
  /**
   * Creates a new SetPTO.
   */
  public SetPTO(boolean climbing) {
    addRequirements(RobotContainer.climber);
    this.climbing = climbing;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.setPTO(climbing);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
