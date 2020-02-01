/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
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

public class Intake extends SubsystemBase {
  private CANSparkMax motor;
  private Solenoid solenoid;
  private double speed = 0.4;

  public Intake() {
    motor = new CANSparkMax(RobotMap.INTAKE, MotorType.kBrushless);
    solenoid = new Solenoid(RobotMap.SOLENOID);
  }

  @Override
  public void periodic() {

  }

  public void stop() {
		motor.set(0.0);
	}
	
	public void intake() {
		motor.set(speed);
	}

	public void reverse() {
		motor.set(-speed);
	}

  public void down() {
    solenoid.set(true);
  } 

  public void up() {
    solenoid.set(false);
  }
}
