// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private CANSparkMax backRightDrive;
  private CANSparkMax frontRightDrive;
  private CANSparkMax frontLeftDrive;
  private CANSparkMax backLeftDrive;
  private CANSparkMax backRightSteering;
  private CANSparkMax frontRightSteering;
  private CANSparkMax frontLeftSteering;
  private CANSparkMax backLeftSteering;
  private CANCoder backRightSensor;
  private CANCoder frontRightSensor;
  private CANCoder frontLeftSensor;
  private CANCoder backLeftSensor;
  private XboxController xboxController;
  private long timeMs = System.currentTimeMillis();
  private ProfiledPIDController backRightModule;
  private ProfiledPIDController frontRightModule;
  private ProfiledPIDController frontLeftModule;
  private ProfiledPIDController backLeftModule;
  private SwerveDriveKinematics kinematics;
  private CANCoder[] sensors;
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    backRightDrive = new CANSparkMax(12, MotorType.kBrushless);
    backRightDrive.setInverted(true);
    backRightSteering = new CANSparkMax(10, MotorType.kBrushless);
    backRightSensor = new CANCoder(11);
    backRightSensor.configMagnetOffset(-106.523);

    frontRightDrive = new CANSparkMax(9, MotorType.kBrushless);
    frontRightDrive.setInverted(true);
    frontRightSteering = new CANSparkMax(7, MotorType.kBrushless);
    frontRightSteering.setInverted(true);
    frontRightSensor = new CANCoder(8);
    frontRightSensor.configMagnetOffset(-95.098);

    frontLeftDrive = new CANSparkMax(6, MotorType.kBrushless);
    frontLeftSteering = new CANSparkMax(4, MotorType.kBrushless);
    frontLeftSensor = new CANCoder(5);
    frontLeftSensor.configMagnetOffset(0);

    backLeftDrive = new CANSparkMax(3, MotorType.kBrushless);
    backLeftSteering = new CANSparkMax(1, MotorType.kBrushless);
    backLeftSensor = new CANCoder(2);
    backLeftSensor.configMagnetOffset(-59.062);

    sensors = new CANCoder[] {
      frontLeftSensor, frontRightSensor, backLeftSensor, backRightSensor
    };

    xboxController = new XboxController(0);

    // double kp = .15;
    double kp = 2.0 / 360.0;
    double ki = 0;
    // double kd = 0;
    double kd = 0;

    // // Module 1 PID

    int maxVelocity = 36000;
    double maxAcceleration = 100000;
    backRightModule = new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    backRightModule.enableContinuousInput(0, 360);
    // Module 2 PID
    frontRightModule = new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    frontRightModule.enableContinuousInput(0, 360);
    // Module 3 PID
    frontLeftModule =  new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    frontLeftModule.enableContinuousInput(0, 360);
    // Module 4 PID
    backLeftModule =  new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    backLeftModule.enableContinuousInput(0, 360);

    Translation2d[] leverarms = new Translation2d[] {
        new Translation2d(.255, .285),
        new Translation2d(.255, -.285),
        new Translation2d(-.255, .285),
        new Translation2d(-.255, -.285)

    };

    kinematics = new SwerveDriveKinematics(leverarms);

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    double controllerLeftY = -xboxController.getLeftY();
    double controllerLeftX = -xboxController.getLeftX();
    double controllerRightX = -xboxController.getRightX();
    double deadBandThresh = .1;


    ChassisSpeeds speeds = new ChassisSpeeds(
      deadbandControllerInput(controllerLeftY, deadBandThresh),
      deadbandControllerInput(controllerLeftX, deadBandThresh),
      deadbandControllerInput(controllerRightX, deadBandThresh)); //GET RID OF RANDOM NUMBERS!!
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    for(int stateNum = 0; stateNum<4; stateNum = stateNum + 1) {
      SwerveModuleState swerveState = states[stateNum];
     states[stateNum] = SwerveModuleState.optimize(swerveState, Rotation2d.fromDegrees(sensors[stateNum].getAbsolutePosition()));
    }

    swerveSteer(backRightSteering, backRightSensor, states[3].angle.getDegrees(), backRightModule, states[3].speedMetersPerSecond, backRightDrive);
    swerveSteer(frontRightSteering, frontRightSensor, states[1].angle.getDegrees(), frontRightModule,  states[1].speedMetersPerSecond, frontRightDrive);
    swerveSteer(frontLeftSteering, frontLeftSensor, states[0].angle.getDegrees(), frontLeftModule,  states[0].speedMetersPerSecond, frontLeftDrive);
    swerveSteer(backLeftSteering, backLeftSensor, states[2].angle.getDegrees(), backLeftModule,  states[2].speedMetersPerSecond, backLeftDrive);

    
    if (System.currentTimeMillis() - timeMs > 100) {
      timeMs = System.currentTimeMillis();
      System.out.println(
        "xboxcontroller right" + xboxController.getRightX() +
        " position2:" + frontRightSensor.getAbsolutePosition() +
        " setpt: " + states[1].angle.getDegrees());
    }

  }

  public void swerveSteer(CANSparkMax steer, CANCoder sensorSwerve, double setpoint, ProfiledPIDController pID,
      double drivespeed, CANSparkMax drive) {
    double position = sensorSwerve.getAbsolutePosition();
    double outputSteer = pID.calculate(position, setpoint);
    steer.set(outputSteer); // TODO CHANGE 
    drive.set(drivespeed);
  }
  public double deadbandControllerInput(double joystickValue,double threshold){
    if  ( Math.abs(joystickValue) < threshold ) {
      return 0;
    }
    else{
      return joystickValue;
    }

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
