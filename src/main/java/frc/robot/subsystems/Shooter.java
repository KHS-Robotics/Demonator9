/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
  private CANSparkMax leader, follower, hood;
  private CANPIDController shooterPid, hoodPid;
  private CANEncoder leaderEnc, followerEnc;

  public Shooter() {
    leader = new CANSparkMax(RobotMap.SHOOTER1, MotorType.kBrushless);
    follower = new CANSparkMax(RobotMap.SHOOTER2, MotorType.kBrushless);
    hood = new CANSparkMax(RobotMap.HOOD, MotorType.kBrushless);

    follower.follow(leader); // TODO: might need inverted
    shooterPid = leader.getPIDController();
    hoodPid = hood.getPIDController();
    leaderEnc = leader.getEncoder();
    followerEnc = follower.getEncoder();

    // TODO: add shuffleboard tab
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot() {
    leader.set(0.25);
  }

  public void stop() {
    leader.set(0.0);
  }

  public void setHood(double angle) {
    hoodPid.setReference(angle, ControlType.kPosition);
  }

  public void enableForClimb() {
    leader.set(0.5);
  }
}
