/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Swerve Module
 */
public class SwerveModule extends SubsystemBase {
  public final String name;

  private boolean isFlipped;
  private final boolean isInverted;

  private final CANSparkMax driveMotor;
  private final CANEncoder driveEncoder;

  private final CANSparkMax pivotMotor;
  private final CANEncoder pivotEncoder;

  private final PIDController pivotPID;
  private final DigitalInput setDetection;
  // private final CANPIDController pivotPID;

  // TODO: speed control for driving
  //private final PIDController m_drivePIDController = new PIDController(0, 0, 0);

  /**
   * Constructs a Swerve Module.
   * @param name the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP P value of Pivot PID
   * @param pivotI I value of Pivot PID
   * @param pivotD D value of Pivot PID
   * @param reversed true if drive motor is reversed
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI, double pivotD, int digitalInputPort, boolean reversed) {
    isInverted = reversed;

    this.name = name;

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(pivotMotorChannel, MotorType.kBrushless);
    pivotMotor.setSmartCurrentLimit(30);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);

    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPositionConversionFactor(360.0 / 18.0); // 360 degree per rotation, 18:1 -> 360 * 1/18

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setVelocityConversionFactor((2 * Math.PI * 0.0508) / 8.31); // 4" diameter wheel (0.0508 meter radius), 8.31:1 -> 2*pi*0.0508 / 8.31

    // pivotPID = pivotMotor.getPIDController();
    // this.setPid(pivotP, pivotI, pivotD);

    pivotPID = new PIDController(pivotP, pivotI, pivotD);
    pivotPID.enableContinuousInput(-180, 180);
    pivotPID.setTolerance(2);

    setDetection = new DigitalInput(digitalInputPort);
  }

  /**
   * Constructs a Swerve Module.
   * @param name the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP P value of Pivot PID
   * @param pivotI I value of Pivot PID
   * @param pivotD D value of Pivot PID
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI, double pivotD, int digitalInputPort) {
    this(name, driveMotorChannel, pivotMotorChannel, pivotP, pivotI, pivotD, digitalInputPort, false);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber(name + " Speed (m/s)", driveEncoder.getVelocity());
    SmartDashboard.putNumber(name + " Angle (Deg)", this.getAngle());
    SmartDashboard.putNumber(name + " Setpoint (Deg)", pivotPID.getSetpoint());
    SmartDashboard.putNumber(name + " Error (Deg)", pivotPID.getPositionError());
    SmartDashboard.putBoolean(name + " atSetpoint", pivotPID.atSetpoint());
    SmartDashboard.putBoolean(name + " isFlipped", this.isFlipped);
    SmartDashboard.putBoolean(name + " isCenter", !this.setDetection.get());
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Sets the PID values for the pivot module.
   * @param p the p value for the pivot module
   * @param i the i value for the pivot module
   * @param d the d value for the pivot module
   */
  public void setPid(double p, double i, double d) {
    pivotPID.setPID(p, i, d);
  }

  /**
   * Sets the desired state for the module.
   * @param state desired state with the speed and angle
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(SwerveModuleState state, boolean useShortestPath) {
    pivotMotor.set(MathUtil.clamp(pivotPID.calculate(getAngle(), useShortestPath ? calculateShortestPath(state.angle.getDegrees()) : state.angle.getDegrees()), -1, 1));
    //driveMotor.set(state.speedMetersPerSecond*(isInverted ? -1 : 1)*(isFlipped && useShortestPath ? -1 : 1)); // TODO: speed control for driving
  }

  /**
   * Sets the desired state for the module.
   * @param state desired state with the speed and angle
   */
  public void setDesiredState(SwerveModuleState state) {
    setDesiredState(state, true);
  }

  /**
   * Sets the desired state for the module.
   * @param speed the desired speed in meters/second
   * @param angle the desired angle in degrees from [-180, 180]
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(double speed, double angle, boolean useShortestPath) {
    setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)), useShortestPath);
  }

  /**
   * Sets the desired state for the module.
   * @param speed the desired speed in meters/second
   * @param angle the desired angle in degrees from [-180, 180]
   */
  public void setDesiredState(double speed, double angle) {
    setDesiredState(speed, angle, true);
  }

  /**
   * Gets the angle of the pivot module.
   * @return the angle of the pivot module ranging from [-180,180]
   */
  public double getAngle() {
    var angle = pivotEncoder.getPosition();
    if(angle > 0) {
      angle %= 360;
      if(angle > 180) {
        angle -= 360;
      }
    }
    else if(angle < 0) {
      angle %= -360;
      if(angle < -180) {
        angle += 360;
      }
    }
    return angle;
  }

  /**
   * Stops the module.
   */
  public void stop() {
    driveMotor.set(0);
    pivotMotor.set(0);
    pivotPID.reset();
  }

  /**
   * For homing the modules
   * @return true if all modules are homed
   */
  public boolean resetEncoder() {
    if(setDetection.get()) { // sensor inverted
      pivotMotor.set(0.025);
      return false;
    } else {
      pivotMotor.set(0);
      pivotEncoder.setPosition(0.0);
      return true;
    }
  }

  /**
   * Calculates the shortest path the pivot module should take, it
   * might be the given <code>targetAngle</code>. Flips the drive motor
   * if there is a shorter path.
   * @param targetAngle the desired angle of the module
   * @return the shortest path to the target angle, flips the
   * drive motor if there is a shorter path
   */
  private double calculateShortestPath(double targetAngle) {
    var currentAngle = this.getAngle();
    var dAngle = Math.abs(targetAngle - currentAngle);
    
    isFlipped = dAngle > 90 && dAngle < 270;
    
    if(isFlipped) {
      if(targetAngle > 0) {
        targetAngle -= 180;
      }
      else if(targetAngle < 0) {
        targetAngle += 180;
      }
    }
    
    return targetAngle;
  }
}
