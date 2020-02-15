/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax motor;
  private CANEncoder motorEnc;
  private DoubleSolenoid solenoid;
  private double speed = 0.35; //Probably Changing ~0.4
  private boolean intaking, releasing;

  public Intake() {
    motor = new CANSparkMax(RobotMap.INTAKE, MotorType.kBrushless);
    solenoid = new DoubleSolenoid(RobotMap.INTAKE_SOLENOID_1, RobotMap.INTAKE_SOLENOID_2);
    motorEnc = motor.getEncoder();

    motor.setIdleMode(IdleMode.kBrake);

    motorEnc.setPositionConversionFactor(1.0 / 2.0);
    motorEnc.setVelocityConversionFactor(1.0 / 2.0);

    var tab = Shuffleboard.getTab("Intake");
    tab.addNumber("Velocity", motorEnc::getVelocity);
    tab.addBoolean("Intaking", () -> intaking);
    tab.addBoolean("Releasing", () -> releasing);
    //tab.addBoolean("Down", () -> solenoid.get());

    up();
  }

  @Override
  public void periodic() {

  }

  public void stop() {
    intaking = false;
    releasing = false;
		motor.set(0.0);
	}
	
	public void intake() {
    if (intaking) 
      return;
    
    intaking = true;
    releasing = false;
		motor.set(speed); // TODO: might want to use velocity pid
	}

	public void reverse() {
    if (releasing) 
      return;
    
    intaking = false;
    releasing = true;
		motor.set(-speed);
	}

  public void down() {
    solenoid.set(Value.kForward);
  } 

  public void up() {
    solenoid.set(Value.kReverse);
  }
}
