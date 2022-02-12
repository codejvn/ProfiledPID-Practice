// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kGoDistance = "Go Distance";
  private static final String kTurnToAngle = "Turn to Angle";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  VictorSP left;
  VictorSP right;
  ProfiledPIDController goDistance;
  Joystick joystick;
  Encoder rightEncoder;
  Encoder leftEncoder;
  double motorSpeed;
  DifferentialDrive drive;
  ADXRS450_Gyro gyro;
  private double tolerance;
  private double rightMotors;
  private double leftMotors;
  double targetAngle;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Go Distance", kGoDistance);
    m_chooser.addOption("Turn to Angle", kTurnToAngle);
    SmartDashboard.putData("Auto choices", m_chooser);
    left = new VictorSP(1);
    right = new VictorSP(0);
    rightEncoder = new Encoder(0, 1);
    leftEncoder = new Encoder(2,3);
    drive = new DifferentialDrive(left, right);
    gyro = new ADXRS450_Gyro();
    
    
    goDistance = new ProfiledPIDController(
    // gains
    1.3, 0, 0.7, 
    // constraints (max Velocity and max Acceleration, respectively)
    new TrapezoidProfile.Constraints(2, 4));
    joystick = new Joystick(0);

    right.setInverted(true);
    rightEncoder.setDistancePerPulse(((Math.PI * 6) / 360) * 0.0254);
    leftEncoder.setDistancePerPulse(((Math.PI * 6) / 360) * 0.0254);  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    rightEncoder.reset();
    leftEncoder.reset();
    System.out.println(rightEncoder);
    System.out.println(leftEncoder);
    
   
    goDistance.reset(0);
    gyro.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kGoDistance:
        // --- GO DISTANCE W/O DRIFT (FUNCTIONAL) ---
        // no drift
        motorSpeed = -(goDistance.calculate(rightEncoder.getDistance(), 10));
        if (Math.abs(gyro.getAngle()) < 1) {
          left.set(motorSpeed);
          right.set(motorSpeed);
          System.out.println("straight");
        }

        else {
          // drifting left
          if (gyro.getAngle() < 0.5) {
            left.set(motorSpeed + (Math.abs(gyro.getAngle()) / 5));
            right.set(motorSpeed);
            System.out.println("left drift");
          }
          
          // drifting right
          else if (gyro.getAngle() > 0) {
            left.set(motorSpeed);
            right.set(motorSpeed + (Math.abs(gyro.getAngle()) / 5));
            System.out.println("right drift");
          }
        }
        System.out.println("power: " + motorSpeed + ", distance: " + rightEncoder.getDistance() + ", gyro angle: " + gyro.getAngle());
        break;
      case kTurnToAngle:
        // --- TURN TO ANGLE ---
        targetAngle = -217; // change this value to any angle
        if ((!(-5 < (targetAngle - gyro.getAngle()) && (targetAngle - gyro.getAngle()) < 5))) {
          if (targetAngle > gyro.getAngle()) {
            rightMotors = -((targetAngle - gyro.getAngle()) / 65);
            leftMotors = ((targetAngle - gyro.getAngle()) / 65);
          }
      
          if (targetAngle < gyro.getAngle()) {
            rightMotors = ((gyro.getAngle() - targetAngle) / 65);
            leftMotors = -((gyro.getAngle() - targetAngle) / 65);
          }

          // if (leftMotors >= 1) {
          //   // System.out.println("l too high");
          //   leftMotors = 1;
          // }
        
          // if (rightMotors >= 1) {
          //   // System.out.println("r too high");
          //   rightMotors = 1;
          // }
        
          // if (leftMotors <= -1) {
          //   // System.out.println("l too low");
          //   leftMotors = -1;
          // }
        
          // if (rightMotors <= -1) {
          //   // System.out.println("r too low");
          //   rightMotors = -1;
          // }

          left.set(leftMotors);
          right.set(rightMotors);

          System.out.println("angle: " + gyro.getAngle() + ", left: " + leftMotors + ", right: " + rightMotors);

        }

        else {
          left.set(0);
          right.set(0);
          break;
        }
        
      case kDefaultAuto:
      default:
        // Put default auto code here

    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    rightEncoder.reset();
    leftEncoder.reset();
    gyro.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(joystick.getY(), joystick.getX());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    System.out.println("disabled");
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
