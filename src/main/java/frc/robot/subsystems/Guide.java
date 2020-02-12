/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Guide extends SubsystemBase {
  Solenoid pnuematic;

  /**
   * Creates a new Guide.
   */
  public Guide() {
    pnuematic = new Solenoid(RobotMap.GUIDE);
  }

  public void guideDown() {
    pnuematic.set(true);

  }

  public void guideUp() {
    pnuematic.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
