/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleConsumer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Uses the Profiled PID class for precise auto movement.
 * 
 * @author Ian Keller, Jeevan Sailesh, Vidur Jannpureddy
 */
public class NemesisProfiledPID implements Controller {

    // controller
    private ProfiledPIDController controller;

    // Profile gains
    private double kP;
    private double kI;
    private double kD;
    private double maxVel;
    private double maxAcc;

    // output of the calculation
    private double command;
    
    // These objects allow us to read from and write
    // to the sensors and motor, respectively
    private DoubleSupplier source;
    private DoubleConsumer output;

    // start and end points and tolerance of the profile
    private double endPoint;
    // termination of the controller
    private boolean done;

    /**
     * Nemesis Profiled PID uses the Profiled PID class for precise auto movement.
     * 
     * @param kP        proportional feedback
     * @param kI        integral feedback
     * @param kD        derivitive feedback
     * @param maxVel    desired max velocity
     * @param maxAcc    desired max acceleration
     * @param tolerance acceptable range to stop controller
     * @param source    sensor source (Encoder, Gyro, Potentiometer, etc)
     * @param output    motor output (TalonSRX, VictorSPX, CANSparkMax, etc)
     */
    public NemesisProfiledPID(double kP, double kI, double kD, double maxVel, double maxAcc, double tolerance,
            DoubleSupplier source, DoubleConsumer output) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        this.source = source;
        this.output = output;
        done = true;

        endPoint = 0.0;

        controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVel, maxAcc));
        controller.setTolerance(tolerance);
        controller.reset(0);
    }

    /**
     * Sets the desired position of the profile
     * 
     * @param setpoint The desired position of the profile
     */
    public void setSetpoint(double setpoint) {
        endPoint = setpoint;
        controller.setGoal(endPoint);
        done = false;
    }

    /**
     * Calculates the outputs of the profile and writes to the motor automatically
     * 
     * @return Motor speed if not done, otherwise return 0
     */
    
    public void calculate() {
        if (!controller.atGoal() && !done) {
            command = controller.calculate(source.getAsDouble(), endPoint);
            output.accept(command);
        }
        
        else {
            output.accept(0);
        }
    }

    /**
     * @return Whether or not the profile has finished
     */
    public boolean isDone() {
        return controller.atGoal();
    }

    /**
     * Ends and resets the controller
     */
    public void endProfile() {
        controller.reset(0);
        done = true;
    }

    /**
     * Change the max velocity of the profile
     * 
     * @param maxVel new max velocity
     */
    public void setMaxVel(double maxVel) {
        this.maxVel = maxVel;
    }

    /**
     * Change the max acceleration of the profile
     * 
     * @param maxAcc new max acceleration
     */
    public void setMaxAcc(double maxAcc) {
        this.maxAcc = maxAcc;
    }
}
