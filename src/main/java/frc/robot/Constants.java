/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Constants {
  // If you are using multiple modules, make sure to define both the port
  // number and the module.
  public static final double FRONT_LEFT_P = 2.0;
  public static final double FRONT_LEFT_I = 0.005;
  public static final double FRONT_LEFT_D = 0.09;
  
  public static final double FRONT_RIGHT_P = 2.0;
  public static final double FRONT_RIGHT_D = 0.005;
  public static final double FRONT_RIGHT_I = 0.09;

  public static final double REAR_LEFT_P = 2.0;
  public static final double REAR_LEFT_I = 0.005;
  public static final double REAR_LEFT_D = 0.09;

  public static final double REAR_RIGHT_P = 2.0;
  public static final double REAR_RIGHT_I = 0.005;
  public static final double REAR_RIGHT_D = 0.09;

  public static final double TARGET_P = 0.0275;
  public static final double TARGET_I = 0.0;
  public static final double TARGET_D = 0.015;
  
}
