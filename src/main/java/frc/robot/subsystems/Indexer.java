/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {
  private CANSparkMax motor;
  private CANEncoder motorEnc;
  private DigitalInput input1, input2, input3, input4, input5;
  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    motor = new CANSparkMax(RobotMap.INDEX_MOTOR, MotorType.kBrushless);
    motorEnc = motor.getEncoder();

    input1 = new DigitalInput(RobotMap.LIMIT_PORT_1);
    input2 = new DigitalInput(RobotMap.LIMIT_PORT_2);
    input3 = new DigitalInput(RobotMap.LIMIT_PORT_3);
    input4 = new DigitalInput(RobotMap.LIMIT_PORT_4);
    input5 = new DigitalInput(RobotMap.LIMIT_PORT_5);

    var tab = Shuffleboard.getTab("Indexer");
    tab.addNumber("Motor Speed", motorEnc::getVelocity);
    tab.addBoolean("Limit Switch 1", input1::get);
    tab.addBoolean("Limit Switch 2", input2::get);
    tab.addBoolean("Limit Switch 3", input3::get);
    tab.addBoolean("Limit Switch 4", input4::get);
    tab.addBoolean("Limit Switch 5", input5::get);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor(double speed) {
    motor.set(speed);
  }
}
