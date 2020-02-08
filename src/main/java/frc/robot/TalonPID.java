/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TalonPID extends CommandBase {
  /**
   * Creates a new TalonPID.
   */
  int setPoint[] = new int[2];
  int leftCount;
  int rightCount;
  long startDeadBand = 0;
  int deadbandThreshold = 100;
  public TalonPID(int distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_chassis);
    
    Robot.m_chassis.rightMotorLead.setInverted(true);
    Robot.m_chassis.rightMotorFollow.setInverted(true);
    setPoint[0] = Robot.m_chassis.leftMotorLead.getSelectedSensorPosition() + distance;
    setPoint[1] =  Robot.m_chassis.rightMotorLead.getSelectedSensorPosition() + distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_chassis.rightMotorLead.config_kP(0, 0.00025);
    Robot.m_chassis.leftMotorLead.config_kP(0, 0.00025);

    Robot.m_chassis.rightMotorLead.configPeakOutputForward(0.4);
    Robot.m_chassis.leftMotorLead.configPeakOutputForward(0.4);

    Robot.m_chassis.rightMotorLead.configClosedloopRamp(0.4);
    Robot.m_chassis.leftMotorLead.configClosedloopRamp(0.4);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_chassis.leftMotorLead.set(ControlMode.Velocity, 300);
    Robot.m_chassis.rightMotorLead.set(ControlMode.Position, 300);

    leftCount = Robot.m_chassis.leftMotorLead.getSelectedSensorPosition();
    rightCount = Robot.m_chassis.rightMotorLead.getSelectedSensorPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_chassis.leftMotorLead.set(0);
    Robot.m_chassis.rightMotorLead.set(0);

    Robot.m_chassis.rightMotorLead.setInverted(false);
    Robot.m_chassis.rightMotorFollow.setInverted(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(leftCount > setPoint[0] - deadbandThreshold && leftCount < setPoint[0] + deadbandThreshold){
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
