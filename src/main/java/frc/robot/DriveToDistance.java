/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToDistance extends CommandBase {
  /**
   * Creates a new DriveToDistance.
   */
  double distance;
  public DriveToDistance(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = distance;
    addRequirements(Robot.m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_chassis.leftMotorLead.setNeutralMode(NeutralMode.Brake);
    Robot.m_chassis.rightMotorLead.setNeutralMode(NeutralMode.Brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_chassis.move(0.7, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_chassis.move(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.m_chassis.leftMotorLead.getSelectedSensorPosition() > distance;
  }
}
