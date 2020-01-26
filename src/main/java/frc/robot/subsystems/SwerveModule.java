/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase {
  private boolean isInverted;
  public final String name;

  private final CANSparkMax driveMotor;
  private final CANSparkMax pivotMotor;

  private final CANPIDController pivotPID;

  private final CANEncoder pivotEncoder;
  private final CANEncoder driveEncoder;

  //private final PIDController m_drivePIDController = new PIDController(1 / 10.0, 0, 0);

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position (Degrees) of " + name, getAngle());
    SmartDashboard.putNumber("Speed of " + name, driveEncoder.getVelocity());
  }

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param aiPort              Port num of the Analog Input
   * @param turnP               P Val of Turn PID
   * @param turnI               I Val of Turn PID
   * @param TurnD               D Val of Turn PID
   * @param encA                Port of Encoder A Channel
   * @param encB                Port of Encoder B Channel
   * @param offset              Offset of the Pivot Motor, 0 by default
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double turnP, double turnI,
      double turnD, boolean reversed) {
    isInverted = reversed;

    this.name = name;

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(pivotMotorChannel, MotorType.kBrushless);
    pivotMotor.setSmartCurrentLimit(30);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);

    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPositionConversionFactor(360.0 / 18.0);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setVelocityConversionFactor((2 * Math.PI * 2) / 8.31); // 4" wheel

    pivotPID = pivotMotor.getPIDController();

    pivotPID.setP(turnP);
    pivotPID.setI(turnI);
    pivotPID.setD(turnD);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //driveEncoder.setDistancePerPulse(distancePerPulse);

    // // Set the distance (in this case, angle) per pulse for the turning encoder.
    // // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // // divided by the encoder resolution.
    // ai.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //pivotPID.enableContinuousInput(MIN_VOLTAGE, MAX_VOLTAGE);
  }

  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pVal, double iVal, double dVal) {
    this(name, driveMotorChannel, pivotMotorChannel, pVal, iVal, dVal, false);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(Math.toRadians(getAngle())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */

  public void setPid (double p, double i, double d) {
    pivotPID.setP(p);
    pivotPID.setI(i);
    pivotPID.setD(d);
  }

  public double setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    //final var driveOutput = m_drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);
    double targetAngle = to360(state.angle.getDegrees());
    double dAngle = Math.abs(to360(getAngle()) - targetAngle);
    
    boolean isFlipped = dAngle >= 90 && dAngle <= 270;

    // Calculate the turning motor output from the turning PID controller.
    //final var turnOutput = pivotPID.calculate(ai.getAverageVoltage(), degreesToVolts((state.angle.getDegrees() + 360.0 + (isFlipped ? 180.0 : 0)) % 360.0));

    // Calculate the turning motor output from the turning PID controller.
    double driveCalculation = state.speedMetersPerSecond;

    if(isFlipped) {
      driveCalculation = -driveCalculation;
    }
    if(isInverted) {
      driveCalculation = -driveCalculation;
    }

    driveMotor.set(driveCalculation);
    pivotPID.setReference(targetAngle, ControlType.kPosition);

    return state.angle.getDegrees();
  }

  public double getAngle() {
    var angle = pivotEncoder.getPosition();
    angle %= 360;

    if(angle > 180) {
      angle -= 360;
    }
    if(angle < -180) {
      angle += 360;
    }

    return -angle;
  }

  private double to360(double angle) {
    if (angle < 0) {
      angle = -angle;
    } else if (angle > 0) {
      angle = -angle;
      angle += 360;
    }
    return angle;
  }
}