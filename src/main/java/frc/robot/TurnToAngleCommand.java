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

public class TurnToAngleCommand extends CommandBase {
  /**
   * Creates a new Pid.
   */
  int deadbandThreshold = 5;
  double P = 0.0101;// for 120 0.0081;
  double I = 0.005;// for 120 0.003;
  double D = 0.005;// for 120 0.003;
  double startDeadBand;
  int integral = 0;
  double finalAngle = 0;
  double previous_error;
  public TurnToAngleCommand(int angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_chassis);
    finalAngle = Robot.m_chassis.gyro.getAngle() + angle;
  }
  

  // Called when the command is initially scheduled.

 /**
   * @param setpoint the setpoint to set
   */
  
  public double PID(){
    double error = finalAngle - Robot.m_chassis.gyro.getAngle();
    
    integral += error * 0.02;   //Assuming clock is constant and doesn't fluctuate

    
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
    
    
    Robot.m_chassis.move(/*Math.min(Math.max(PID(), -0.4), 0.4)*/0, PID(), true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double angle = Robot.m_chassis.gyro.getAngle();

    System.out.println(angle - finalAngle);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angle = Robot.m_chassis.gyro.getAngle();
    
    if(angle > finalAngle - deadbandThreshold && angle < finalAngle + deadbandThreshold){
      if(System.currentTimeMillis() - startDeadBand > 500){
        System.out.println(angle - finalAngle);
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
