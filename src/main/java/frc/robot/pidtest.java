// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class pidtest {
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //@Override
  public static void main(String[] args){
    ProfiledPIDController pid = new ProfiledPIDController(1.3, 0, 0, new TrapezoidProfile.Constraints(1, 2));
    PIDController pid_control = new PIDController(1.3, 0, 0);
    TrapezoidProfile.State goals = new TrapezoidProfile.State(10, 0);
    pid.reset(0);
    pid.setGoal(goals);
    double distance = 0;
    pid.disableContinuousInput();
    for (int i = 0; i < 21; i++){
      System.out.println("The current goal: " + distance);
      System.out.println("Regular PID: " + pid_control.calculate(distance, 10));
      System.out.println("The goal: " + pid.getGoal());
      System.out.println("Profiled PID: " + pid.calculate(distance, goals) + "\n");
      distance = distance + 0.5;
    }
    
  }

}