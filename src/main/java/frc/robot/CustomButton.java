/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Button;

public class CustomButton extends Button {
  /**
   * Creates a new CustomButton.
   */
  public CustomButton(BooleanSupplier supplier) {
    super(supplier);
  }
}
