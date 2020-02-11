/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ChassisDefaultCommand extends CommandBase {
  /**
   * Creates a new ChassisDefaultCommand.
   */
  public ChassisDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_chassis);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double speed = Robot.m_oi.getMove();
    // double turn = Robot.m_oi.getTurn();
    if(Robot.m_oi.isReset()){
      Robot.m_chassis.leftMotorLead.setSelectedSensorPosition(0);
      Robot.m_chassis.rightMotorLead.setSelectedSensorPosition(0);
    }
    if(Robot.m_oi.isGoDistance()){
      int distance = (int)SmartDashboard.getNumber("Distance", 5000);
      CommandScheduler.getInstance().schedule(new Pid(distance));
    }
    Robot.m_chassis.move(Robot.m_oi.getMove(), Robot.m_oi.getTurn(), true);

    if(Robot.m_oi.goPid()){
      int distance = (int)SmartDashboard.getNumber("Distance", 5000);

      CommandScheduler.getInstance().schedule(new PidDrive(distance,distance));
    }
    if(Robot.m_oi.turn()){
      CommandScheduler.getInstance().schedule(new TurnToAngleCommand(90));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
