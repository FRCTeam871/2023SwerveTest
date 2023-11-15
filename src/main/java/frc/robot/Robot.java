// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
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
  private long timeMs=System.currentTimeMillis();
  private PIDController module1Steer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

      driveMotor1 = new CANSparkMax(12, MotorType.kBrushless);
      steeringMotor1 = new CANSparkMax(10,MotorType.kBrushless);
      steeringMotor1.setInverted(true);
      sensor1 = new CANCoder(11);
      sensor1.configMagnetOffset(0);
      
      driveMotor2 = new CANSparkMax(9, MotorType.kBrushless);
      steeringMotor2 = new CANSparkMax(7,MotorType.kBrushless);
      sensor2 = new CANCoder(8);
      sensor2.configMagnetOffset(282.22);

      driveMotor3 = new CANSparkMax(6, MotorType.kBrushless);
      steeringMotor3 = new CANSparkMax(4,MotorType.kBrushless);
      sensor3 = new CANCoder(5);
      sensor3.configMagnetOffset(53);

      driveMotor4 = new CANSparkMax(3, MotorType.kBrushless);
      steeringMotor4 = new CANSparkMax(1,MotorType.kBrushless);
      sensor4 = new CANCoder(2);
      sensor4.configMagnetOffset(54.32);

      xboxController = new XboxController(0);
      module1Steer = new PIDController(1.0/360.0, 1.0/3600, 0);

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

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
   // double drivePower = xboxController.getLeftY();  
   // double steeringPower = 0; //xboxController.getRightX();
   // driveMotor1.set(drivePower);
    //steeringMotor1.set(steeringPower);
   // driveMotor2.set(drivePower);
   // steeringMotor2.set(steeringPower);
   // driveMotor3.set(drivePower);
   // steeringMotor3.set(steeringPower);
    //driveMotor4.set(drivePower);
   //steeringMotor4.set(steeringPower);

   double setpoint = Math.atan2(xboxController.getLeftX(), xboxController.getLeftY())*180/Math.PI;

   double position = sensor1.getAbsolutePosition();
   double output = module1Steer.calculate(position, setpoint);
   steeringMotor1.set(output);

    if(System.currentTimeMillis()-timeMs>100) {
      timeMs=System.currentTimeMillis();
      System.out.println("position:" + sensor1.getAbsolutePosition() + " position2:" + sensor2.getAbsolutePosition() + " position3:" + sensor3.getAbsolutePosition() + " position4:" + sensor4.getAbsolutePosition() + "setpoint=" + setpoint); 
    }
    
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
