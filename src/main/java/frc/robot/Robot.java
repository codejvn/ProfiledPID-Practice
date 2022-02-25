// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.hal.simulation.NotifyCallback;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import java.util.List;
import edu.wpi.first.hal.HALValue;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import java.util.function.DoubleSupplier;
// import java.util.function.DoubleConsumer;
// import edu.wpi.first.math.VecBuilder;


//MAYBE check in launch.json for issues

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kGoDistance = "Go Distance";
  private static final String kTimerAuto = "Timer";
  // private static final String kTurnToAngle = "Turn to Angle";
  // private static final String kControllerTest = "Controller Test";
  String m_autoSelected;
  SendableChooser<String> m_chooser = new SendableChooser<>();
  VictorSP left;
  VictorSP right;
  ProfiledPIDController goDistance;
  Joystick joystick;
  Encoder rightEncoder;
  Encoder leftEncoder;
  EncoderSim rightEncoderSim;
  EncoderSim leftEncoderSim;
  double motorSpeed;
  DifferentialDrive drive;
  ADXRS450_Gyro gyro;
  ADXRS450_GyroSim gyroSim;
  // private double tolerance;
  double targetAngle;
  // private NemesisProfiledPID customProfiledPID;
  // private DoubleSupplier input;
  // private DoubleConsumer output;
  private DifferentialDrivetrainSim driveSim;
  private DifferentialDriveOdometry odometry;
  private Timer timer;
  // private int counter;
  private Trajectory m_trajectory;
  // private PIDController pidController;
  // private DifferentialDriveKinematics kinematics;
  // private DifferentialDriveWheelSpeeds wheelSpeeds;
  // private ChassisSpeeds chassisSpeeds;
  CallbackStore leftStore;
  CallbackStore rightStore;
  Field2d field;

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Go Distance", kGoDistance);
    m_chooser.addOption("Timer", kTimerAuto);
    // m_chooser.addOption("Turn to Angle", kTurnToAngle);
    // m_chooser.addOption("Controller Test", kControllerTest);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_autoSelected = m_chooser.getSelected();
    left = new VictorSP(1);
    right = new VictorSP(0);
    rightEncoder = new Encoder(0, 1);
    leftEncoder = new Encoder(2, 3);
    rightEncoderSim = new EncoderSim(rightEncoder);
    leftEncoderSim = new EncoderSim(leftEncoder);
    drive = new DifferentialDrive(left, right);
    gyro = new ADXRS450_Gyro();
    gyroSim = new ADXRS450_GyroSim(gyro);
    // input = () -> rightEncoder.getDistance();
    // output = a -> motorSpeed = a;
    // customProfiledPID = new NemesisProfiledPID(1.3, 0, 0.7, 1.0, 2.0, 0.5, input, output);
    // dcMotor = new DCMotor(3.0, 2.425, 133, 2.7, 556.0632, 1);
    driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
      7.29,                    // 7.29:1 gearing reduction.
      7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
      60.0,                    // The mass of the robot is 60 kg.
      Units.inchesToMeters(3), // The robot uses 3" radius wheels.
      0.7112,                  // The track width is 0.7112 meters.
      null);

      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    // counter = 0;
    
    goDistance = new ProfiledPIDController(
    // gains
    1.3, 0, 0.7, 
    // constraints (max Velocity and max Acceleration, respectively)
    new TrapezoidProfile.Constraints(1, 2));
    joystick = new Joystick(0);

    right.setInverted(true);
    rightEncoder.setDistancePerPulse(((Math.PI * 6) / 360) * 0.0254);
    leftEncoder.setDistancePerPulse(((Math.PI * 6) / 360) * 0.0254);  
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    timer = new Timer();
    // kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
    // wheelSpeeds = new DifferentialDriveWheelSpeeds();
    // chassisSpeeds = new ChassisSpeeds();
    
    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            
new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 2), new Translation2d(2, 2)),
            new Pose2d(3, 1, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Create and push Field2d to SmartDashboard.
    field = new Field2d();
    SmartDashboard.putData(field);

    // Push the trajectory to Field2d.
    field.getObject("traj").setTrajectory(m_trajectory);
  }

  /*
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /*
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
    rightEncoder.reset();
    leftEncoder.reset();
    goDistance.reset(0);
    gyro.reset();
    timer.reset();
    timer.start();
  }

  /* This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { 
    m_autoSelected = m_chooser.getSelected();   
    switch (m_autoSelected) {
      case kGoDistance:
        // --- GO DISTANCE W/O DRIFT (FUNCTIONAL) ---
        // /*FOLLLOWING IS WORKING CODE*/
        System.out.println("go distance auto".toUpperCase());
        motorSpeed = goDistance.calculate(rightEncoder.getDistance(), 10);
        if (Math.abs(gyro.getAngle()) < 1) {
          left.set(motorSpeed / 12);
          right.set(motorSpeed / 12);
          // System.out.println("straight");
        }
 
        else {
          // drifting left
          if (gyro.getAngle() < 0.5) {
            left.set(motorSpeed + (Math.abs(gyro.getAngle()) / 5));
            right.set(motorSpeed);
            // System.out.println("left drift");
          }
           
          // drifting right
          else if (gyro.getAngle() > 0.5) {
            left.set(motorSpeed);
            right.set(motorSpeed + (Math.abs(gyro.getAngle()) / 5));
            // System.out.println("right drift");
          }
        }
        break;
        // distance and gyro values twitch/oscillate after auton is done
        
      case kTimerAuto:
        System.out.println("timer auto".toUpperCase());  
        if (timer.get() > 5.0 && timer.get() < 10.0) {
          drive.arcadeDrive(0.5, 0);
        }
        break;
        //mostly only for simulation testing purposes (needed a super simple auto to start)

      case kDefaultAuto:
      default:
        System.out.println("default auto");
        // put default auto code here

        break;
    }
      // auto chooser doesn't work, can select auto with smart dashboard but once auton is
      // enabled, the simulation window closes out
      // not sure if this is a code problem, something we need to read more about, or a simulation bug 
  }

  /* This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    rightEncoder.reset();
    leftEncoder.reset();
    gyro.reset();
  }

  /* This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(joystick.getY(), joystick.getX());
  }

  /* This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    System.out.println("disabled fr");
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /* This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}
  
  /* This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /* This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
  @Override
  public void simulationInit() {
    NotifyCallback callback = (String name, HALValue value) -> {
      if (value.getType() == HALValue.kInt) {
        System.out.println("Value of " + name + " is " + value.getLong());
      }
      else {
        System.out.println("Recieved different type");
      }
    };
    leftStore = leftEncoderSim.registerCountCallback(callback, false);
    rightStore = rightEncoderSim.registerCountCallback(callback, false);
  }
  
  /*
   * Simulator periodic runs every 20 ms, and updates the values in the simulator. 
   */
  int i = 0;
  @Override
  public void simulationPeriodic(){ 
    i++;
    driveSim.setInputs(left.get() * RobotController.getInputVoltage(),
    right.get() * RobotController.getInputVoltage());
      
    if ((i % 20) == 0) {
      System.out.println("Right motor percent: " + right.get() + ", left motor percent " + left.get() + ", getInputVoltage: " + RobotController.getInputVoltage());
      System.out.println("Current Amps: " + driveSim.getCurrentDrawAmps());
      System.out.println("position left  sim robot: " + driveSim.getLeftPositionMeters() + " position right sim robot: " + driveSim.getRightPositionMeters());
    }
  
    driveSim.update(0.02);

  
    // Update all of our sensors.
    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-driveSim.getHeading().getDegrees());

    SmartDashboard.putData("Field", field);
    field.setRobotPose(odometry.getPoseMeters());

     // updating odometry
    odometry.update(gyro.getRotation2d(), leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
  }
}