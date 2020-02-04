/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.vision.PixyCam;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CPManipulator extends SubsystemBase {
  private CANSparkMax motor;
  private double speed;
  private final String[] colors = {"R", "G", "B", "Y"};

  public CPManipulator() {
    motor = new CANSparkMax(RobotMap.MANIPULATOR, MotorType.kBrushless);

    var tab = Shuffleboard.getTab("Manipulator Speed");
    tab.addNumber("Speed", () -> getSpeed());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin(double speed) {
    motor.set(speed);
    this.speed = speed;
  }

  public double getSpeed() {
    return speed;
  }
}
