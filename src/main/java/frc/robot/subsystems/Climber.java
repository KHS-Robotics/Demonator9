/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

  private CANSparkMax telescope;
  private Solenoid pto;
  private double speed;
  
  public Climber() {
    telescope = new CANSparkMax(RobotMap.TELESCOPE, MotorType.kBrushless);
    pto = new Solenoid(RobotMap.PTO);
    setPTO(false); // TODO: Might be inverted

    // TODO: add shuffleboard tab
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void up() {
    telescope.set(speed);
  }

  public void down() {
    telescope.set(-speed);
  }

  public void setPTO(boolean climbing) {
    pto.set(climbing);
  }
}
