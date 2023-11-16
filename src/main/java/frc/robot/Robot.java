// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  private CANSparkMax driveMotor1;
  private CANSparkMax driveMotor2;
  private CANSparkMax driveMotor3;
  private CANSparkMax driveMotor4;
  private CANSparkMax steeringMotor1;
  private CANSparkMax steeringMotor2;
  private CANSparkMax steeringMotor3;
  private CANSparkMax steeringMotor4;
  private CANCoder sensor1;
  private CANCoder sensor2;
  private CANCoder sensor3;
  private CANCoder sensor4;
  private XboxController xboxController;
  private long timeMs = System.currentTimeMillis();
  private PIDController module1Steer;
  private PIDController module2Steer;
  private PIDController module3Steer;
  private PIDController module4Steer;
  private SwerveDriveKinematics kinematics;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    driveMotor1 = new CANSparkMax(12, MotorType.kBrushless);
    driveMotor1.setInverted(true);
    steeringMotor1 = new CANSparkMax(10, MotorType.kBrushless);
    steeringMotor1.setInverted(true);
    sensor1 = new CANCoder(11);
    sensor1.configMagnetOffset(-12.480);

    driveMotor2 = new CANSparkMax(9, MotorType.kBrushless);
    steeringMotor2 = new CANSparkMax(7, MotorType.kBrushless);
    sensor2 = new CANCoder(8);
    sensor2.configMagnetOffset(79.014);

    driveMotor3 = new CANSparkMax(6, MotorType.kBrushless);
    driveMotor3.setInverted(true);
    steeringMotor3 = new CANSparkMax(4, MotorType.kBrushless);
    sensor3 = new CANCoder(5);
    sensor3.configMagnetOffset(-299.619);

    driveMotor4 = new CANSparkMax(3, MotorType.kBrushless);
    steeringMotor4 = new CANSparkMax(1, MotorType.kBrushless);
    sensor4 = new CANCoder(2);
    sensor4.configMagnetOffset(186.24);

    xboxController = new XboxController(0);
    // Module 1 PID
    module1Steer = new PIDController(1.0 / 360.0, 1.0 / 3600, 0);
    module1Steer.enableContinuousInput(0, 360);
    // Module 2 PID
    module2Steer = new PIDController(1.0 / 360.0, 1.0 / 3600, 0);
    module2Steer.enableContinuousInput(0, 360);
    // Module 3 PID
    module3Steer = new PIDController(1.0 / 360.0, 1.0 / 3600, 0);
    module3Steer.enableContinuousInput(0, 360);
    // Module 4 PID
    module4Steer = new PIDController(1.0 / 360.0, 1.0 / 3600, 0);
    module4Steer.enableContinuousInput(0, 360);

    Translation2d[] leverarms = new Translation2d[] {
        new Translation2d(.255, .285),
        new Translation2d(-.255, .285),
        new Translation2d(-.255, -.285),
        new Translation2d(.255, -.285)

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
    ChassisSpeeds speeds = new ChassisSpeeds(xboxController.getLeftX(), xboxController.getLeftY(), xboxController.getRightX());
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    swerveSteer(steeringMotor1, sensor1, states[0].angle.getDegrees(), module1Steer, states[0].speedMetersPerSecond, driveMotor1);
    swerveSteer(steeringMotor2, sensor2, states[1].angle.getDegrees(), module2Steer,  states[1].speedMetersPerSecond, driveMotor2);
    swerveSteer(steeringMotor3, sensor3, states[2].angle.getDegrees(), module3Steer,  states[2].speedMetersPerSecond, driveMotor3);
    swerveSteer(steeringMotor4, sensor4, states[3].angle.getDegrees(), module4Steer,  states[3].speedMetersPerSecond, driveMotor4);
    if (System.currentTimeMillis() - timeMs > 100) {
      timeMs = System.currentTimeMillis();
      System.out.println(
          "position:" + sensor1.getAbsolutePosition() + " position2:" + sensor2.getAbsolutePosition() + " position3:"
              + sensor3.getAbsolutePosition() + " position4:" + sensor4.getAbsolutePosition());
    }

  }
  public void swerveSteer(CANSparkMax steer, CANCoder sensorSwerve,double setpoint, PIDController pID,double drivespeed,CANSparkMax drive) {
    double position = sensorSwerve.getAbsolutePosition();
    double outputSteer = pID.calculate(position, setpoint);
    steer.set(outputSteer);
    drive.set(drivespeed);
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
