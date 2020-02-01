/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChassisSubsystem extends SubsystemBase {
  /**
   * Creates a new ChassisSubsystem.
   */
  WPI_TalonSRX leftMotorLead = new WPI_TalonSRX(3);
  WPI_TalonSRX leftMotorFollow = new WPI_TalonSRX(4);
  
  WPI_TalonSRX rightMotorLead = new WPI_TalonSRX(1);
  WPI_TalonSRX rightMotorFollow = new WPI_TalonSRX(2);

  DifferentialDrive driver;

  public ChassisSubsystem() {
    // leftMotorLead.setSafetyEnabled(false);
    // rightMotorLead.setSafetyEnabled(false);
    // leftMotorFollow.setSafetyEnabled(false);
    // rightMotorFollow.setSafetyEnabled(false);

    leftMotorFollow.follow(leftMotorLead);
    rightMotorFollow.follow(rightMotorLead);

    leftMotorLead.config_kP(0, 0.25);
   
    rightMotorLead.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    leftMotorLead.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    leftMotorLead.setSensorPhase(true);

    driver = new DifferentialDrive(leftMotorLead, rightMotorLead);
  }
  
  
  public void move(double speed, double turn){
    driver.arcadeDrive(speed, turn);
  }

  public void UpdateSmartDashBoard(){
    double leftPos = leftMotorLead.getSelectedSensorPosition();
    double rightPos = rightMotorLead.getSelectedSensorPosition();

    SmartDashboard.putNumber("Left Encoder Count", leftPos);
    SmartDashboard.putNumber("Right Encoder Count", rightPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
