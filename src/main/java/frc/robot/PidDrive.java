/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PidDrive extends CommandBase {
  /**
   * Creates a new Pid.
   */
  int deadbandThreshold = 100;
  double P = SmartDashboard.getNumber("P", 0.0003);//0.0003;
  double I = SmartDashboard.getNumber("I", 0.000075);//0.000075;
  double D = SmartDashboard.getNumber("D", 0.00005);//0.00005;
  double startDeadBand;

  //index 0 is left, index 1 is right
  int integrals[] = {0, 0};
  double previous_errors[] = {0, 0};
  int setPoints[] = {0, 0};
  
  int leftTarget;
  int rightTarget;
  
  public PidDrive(int leftTar, int rightTar) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_chassis);
    leftTarget = leftTar;
    rightTarget = rightTar;
    setPoints[0] = leftTar + Robot.m_chassis.leftMotorLead.getSelectedSensorPosition();
    setPoints[1] = rightTar + Robot.m_chassis.rightMotorLead.getSelectedSensorPosition();
  }
  

  // Called when the command is initially scheduled.

 /**
   * @param setpoint the setpoint to set
   */
  
  public double PID(int index){
    double error;
    
    if(index == 0){
      error = setPoints[index] - Robot.m_chassis.leftMotorLead.getSelectedSensorPosition();
    } else {
      error = setPoints[index] - Robot.m_chassis.rightMotorLead.getSelectedSensorPosition();
    }
    
    if(error < 5000){
      integrals[index] += error * 0.02;   //Assuming clock is constant and doesn't fluctuate

    }
    double derivative = (error - previous_errors[index]);

    previous_errors[index] = error;

    double endValue = P * error + I*integrals[index] + D*derivative;


    //need to do something with previous error
    
    return endValue;

    
  } 
  @Override
  public void initialize() {
    Robot.m_chassis.leftMotorLead.configPeakOutputForward(1);
    Robot.m_chassis.rightMotorLead.configPeakOutputForward(1);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Robot.m_chassis.leftMotorLead.set(ControlMode.Velocity, 300);//PID(0) * 500 * 4096 / 600
    // Robot.m_chassis.rightMotorLead.set(ControlMode.Velocity, 300);

      Robot.m_chassis.move(PID(0), PID(0), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_chassis.rightMotorLead.set(0);
    Robot.m_chassis.rightMotorFollow.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int rightCount = Robot.m_chassis.rightMotorLead.getSelectedSensorPosition();
    int leftCount = Robot.m_chassis.leftMotorLead.getSelectedSensorPosition();
    if((leftCount > leftTarget - deadbandThreshold && leftCount < leftTarget + deadbandThreshold) && (rightCount > rightTarget - deadbandThreshold && rightCount < rightTarget + deadbandThreshold)){
      // if(System.currentTimeMillis() - startDeadBand > 1000){
      //   return true;
      // }
      return true;
    } else {
      startDeadBand = System.currentTimeMillis();

    }
    if(Robot.m_oi.endPid()){
      return true;
    }
    
    return false;

  }
}
