/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static final int XBOX_PORT = 0;

  public static final int JOYSTICK_XY = 1;
  public static final int JOYSTICK_Z = 2;

  public static final int FRONT_LEFT_PIVOT = 13;
  public static final int FRONT_RIGHT_PIVOT = 0;
  public static final int REAR_LEFT_PIVOT = 15;
  public static final int REAR_RIGHT_PIVOT = 4;
  public static final int FRONT_LEFT_DRIVE = 3;
  public static final int FRONT_RIGHT_DRIVE = 11;
  public static final int REAR_LEFT_DRIVE = 12;
  public static final int REAR_RIGHT_DRIVE = 14;

  public static final int FRONT_RIGHT_ANALOG = 0;
  public static final int FRONT_LEFT_ANALOG = 1;
  public static final int REAR_RIGHT_ANALOG = 2;
  public static final int REAR_LEFT_ANALOG = 3;

  public static final int FRONT_RIGHT_DRIVE_ENC_A = 0;
	public static final int FRONT_RIGHT_DRIVE_ENC_B = 1;
	public static final int FRONT_LEFT_DRIVE_ENC_A = 2;
	public static final int FRONT_LEFT_DRIVE_ENC_B = 3;
	public static final int REAR_RIGHT_DRIVE_ENC_A = 4;
	public static final int REAR_RIGHT_DRIVE_ENC_B = 5;
	public static final int REAR_LEFT_DRIVE_ENC_A = 6;
	public static final int REAR_LEFT_DRIVE_ENC_B = 7;

  public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
  public static final byte NAVX_UPDATE_RATE_HZ = (byte) 50;
}
