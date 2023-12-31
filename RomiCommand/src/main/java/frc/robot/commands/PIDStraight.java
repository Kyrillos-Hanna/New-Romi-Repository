// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.sensors.RomiGyro;

public class PIDStraight extends CommandBase {

  RomiDrivetrain m_RomiDrivetrain;
  PIDController m_PIDController = new PIDController(0, 0, 0);


  /** Creates a new PIDStraight. */
  public PIDStraight(RomiDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_RomiDrivetrain = drivetrain;
    addRequirements(m_RomiDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_RomiDrivetrain.arcadeDrive(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RomiDrivetrain.arcadeDrive(0.5, m_PIDController.calculate(m_RomiDrivetrain.m_RomiGyro.getAngleZ(), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_RomiDrivetrain.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
