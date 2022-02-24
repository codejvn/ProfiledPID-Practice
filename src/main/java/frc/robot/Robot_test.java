// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.math.controller.PIDController;
// import frc.controllers.NemesisProfiledPID;
// import java.util.function.DoubleSupplier;
// import java.util.function.DoubleConsumer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.hal.HALValue;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

/*maybe start using the diff odometry, as you can use that to get the posiiton, however
look at how it would be implemented in the sim periodic

*/

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot_test extends TimedRobot {
  // private static final String kDefaultAuto = "Default";
  // private static final String kGoDistance = "Go Distance";
  // private static final String kTurnToAngle = "Turn to Angle";
  // private static final String kControllerTest = "Controller Test";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();
  VictorSP left;
  VictorSP right;
  ProfiledPIDController goDistance;
  Joystick joystick;
  Joystick joystick1;
  Encoder rightEncoder;
  Encoder leftEncoder;
  EncoderSim rightEncoderSim;
  EncoderSim leftEncoderSim;
  double motorSpeed;
  DifferentialDrive drive;
  ADXRS450_Gyro gyro;
  ADXRS450_GyroSim gyroSim;
  // private double tolerance;
  // private double rightMotors;
  // private double leftMotors;
  double targetAngle;
  // private NemesisProfiledPID customProfiledPID;
  // private DoubleSupplier input;
  // private DoubleConsumer output;
  private double encoderValue;
  private DifferentialDrivetrainSim driveSim;
  private DCMotor dcMotor;
  private DifferentialDriveOdometry odometry;
  private Timer timer;
  private boolean runSim;
  private int counter;
  private boolean inTele;
  private Trajectory m_Trajectory;
  private PIDController pidController;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveWheelSpeeds wheelSpeeds;
  private ChassisSpeeds chassisSpeeds;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("Go Distance", kGoDistance);
    // m_chooser.addOption("Turn to Angle", kTurnToAngle);
    // m_chooser.addOption("Controller Test", kControllerTest);
    // SmartDashboard.putData("Auto choices", m_chooser);
    left = new VictorSP(1);
    right = new VictorSP(0);
    rightEncoder = new Encoder(0, 1);
    leftEncoder = new Encoder(2,3);
    rightEncoderSim = new EncoderSim(rightEncoder);
    leftEncoderSim = new EncoderSim(leftEncoder);
    drive = new DifferentialDrive(left, right);
    gyro = new ADXRS450_Gyro();
    gyroSim = new ADXRS450_GyroSim(gyro);
    // input = () -> rightEncoder.getDistance();
    // output = a -> motorSpeed = a;
    // customProfiledPID = new NemesisProfiledPID(1.3, 0, 0.7, 1.0, 2.0, 0.5, input, output);
    dcMotor = new DCMotor(3.0, 2.425, 133, 2.7, 556.0632, 1);
    driveSim = new DifferentialDrivetrainSim(dcMotor, 1.0, 7.5, 31.75, 0.08255, 27.5, null);
    counter = 0;
    
    goDistance = new ProfiledPIDController(
    // gains
    1.3, 0, 0.7, 
    // constraints (max Velocity and max Acceleration, respectively)
    new TrapezoidProfile.Constraints(1, 2));
    joystick = new Joystick(0);
    joystick1 = new Joystick(1);

    right.setInverted(true);
    rightEncoder.setDistancePerPulse(((Math.PI * 6) / 360) * 0.0254);
    leftEncoder.setDistancePerPulse(((Math.PI * 6) / 360) * 0.0254);  
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    timer = new Timer();
    runSim = false;
    inTele = false;
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
    wheelSpeeds = new DifferentialDriveWheelSpeeds();
    chassisSpeeds = new ChassisSpeeds();
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
    rightEncoder.reset();
    leftEncoder.reset();
    System.out.println(rightEncoder.getDistance());
    System.out.println(leftEncoder.getDistance());
    goDistance.reset(0);
    gyro.reset();
    timer.reset();
    timer.start();
    rightEncoderSim.resetData();
    leftEncoderSim.resetData();
    gyroSim.setAngle(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // while (timer.get() > 5.0 && timer.get() < 10.0) {
    //   drive.arcadeDrive(0.5, 0);
    // }
    // auton works w/ timer, but encoders continue w/ distance
    // even after motors are off
    
    //System.out.println("autonomousPeriodic");
    // switch (m_autoSelected) {
    //   case kGoDistance:
        // --- GO DISTANCE W/O DRIFT (FUNCTIONAL) ---
        // no drift
        /*FOLLLOWING IS WORKING CODE*/
         encoderValue = SmartDashboard.getNumber("DB/Slider 0", 0);
         runSim = true;
         motorSpeed = -(goDistance.calculate(encoderValue, 10));
         while (counter < 10) {
          if (Math.abs(gyro.getAngle()) < 1) {
            left.set(motorSpeed/12);
            right.set(motorSpeed/12);
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
         motorSpeed = -(goDistance.calculate(encoderValue, 10));
         System.out.println("encoder value:" + encoderValue + "power: " + motorSpeed + ", distance: " + encoderValue + ", gyro angle: " + gyro.getAngle() + ", volts: " + RobotController.getInputVoltage()
         + ", amps: " + RobotController.getInputVoltage());;
         counter = counter + 1;
         System.out.println("Counter value: " + counter);
         }
         runSim = false;
        // when running sim periodic, motor value does a bit of fluctuation until it settles on the correct value
        //however, one thing to note is that while the print statements are giving an output that makes 
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
    inTele = true;
    //drive.arcadeDrive(joystick.getY(), joystick.getX());
    // Using tank drive instead, because the output to the motors relative to the input is more intuitive
    drive.tankDrive(joystick.getY(), joystick1.getY());
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    System.out.println("disabled");
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}
    
    //right.set(0);
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  /**
   * Simulator periodic runs every 20 ms, and updates the values in the simulator. 
   */
  @Override
  public void simulationPeriodic(){ 
   //EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
    //EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
    //System.out.println("starting sim periodic");
    // rightEncoderSim.setDistance(encoderValue);
    // leftEncoderSim.setDistance(encoderValue);
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    leftEncoderSim.setDistance(0);
    rightEncoderSim.setDistance(0);
    while (runSim || inTele){
      driveSim.setInputs(left.get() * RobotController.getInputVoltage(),
      right.get() * RobotController.getInputVoltage());
      //System.out.println("Right motor percent: " + right.get() + ", left motor percent " + left.get() + ", getInputVoltage: " + RobotController.getInputVoltage());
  
      // Advance the model by 20 ms. Note that if you are running this
      // subsystem in a separate thread or have changed the nominal timestep
      // of TimedRobot, this value needs to match it.
      driveSim.update(0.02);

      //updating odometry?
      odometry.update(gyro.getRotation2d(), leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
  
      // Update all of our sensors.
      leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
      leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
      rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
      rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
      gyroSim.setAngle(-driveSim.getHeading().getDegrees());
    }
    
  }
}
