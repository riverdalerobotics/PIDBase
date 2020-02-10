/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Pid extends CommandBase {
  /**
   * Creates a new Pid.
   */
  int deadbandThreshold = 100;
  double P = 0.00025;
  double I = 0.000075;
  double D = 0.00005;
  double startDeadBand;
  int integral,  setpoint = 0;
  double previous_error;
  boolean completeFlag;
  public Pid(int setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_chassis);
    this.setpoint = Robot.m_chassis.rightMotorLead.getSelectedSensorPosition() + setPoint;
    completeFlag = false;
  }
  

  // Called when the command is initially scheduled.

 /**
   * @param setpoint the setpoint to set
   */
  
  public double PID(){
    double error = setpoint - Robot.m_chassis.rightMotorLead.getSelectedSensorPosition();
    if(error < 5000){
      integral += error * 0.02;   //Assuming clock is constant and doesn't fluctuate

    }
    double derivative = (error - previous_error);
    double endValue = P * error + I*integral + D*derivative;
    previous_error = error;
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

    int currentPos = Robot.m_chassis.rightMotorLead.getSelectedSensorPosition();

    int target = currentPos + setpoint;

    
    
    Robot.m_chassis.move(/*Math.min(Math.max(PID(), -0.4), 0.4)*/PID(), 0, true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int encoderCount = Robot.m_chassis.rightMotorLead.getSelectedSensorPosition();
    
    if(encoderCount > setpoint - deadbandThreshold && encoderCount < setpoint + deadbandThreshold){
      if(System.currentTimeMillis() - startDeadBand > 1000){
        return true;
      }
    } else {
      startDeadBand = System.currentTimeMillis();

    }
    if(Robot.m_oi.endPid()){
      return true;
    }
    
    return false;

  }
}
