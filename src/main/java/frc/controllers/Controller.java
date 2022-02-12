/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

/**
 * Basic template for a feedback controller
 */
public interface Controller {

  /**
   * constant timestep value, 20ms refresh rate
   */
  public static final double dt = 0.02;

  /**
   * resets the desired setpoint of the controller
   * 
   * @param setpoint setpoint
   */
  public void setSetpoint(double setpoint);

  /**
   * calculates the output power of the controller based on the current position
   */
  public void calculate();

  /**
   * checks whether the current value is within the required threshold to stop the
   * controller
   * 
   * @return whether the controller has finished its feedback loop
   */
  public boolean isDone();
}
