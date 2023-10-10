// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.RomiDrivetrain;

public class RomiBalance extends CommandBase {

  private RomiDrivetrain m_RomiDrivetrain;
  private boolean isDetected;
  private PIDController m_PIDBalance = new PIDController(0,0,0);

  /** Creates a new RomiBalance. */
  public RomiBalance(RomiDrivetrain drivetrain) {
    m_RomiDrivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_RomiDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_RomiDrivetrain.m_RomiGyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_RomiDrivetrain.m_RomiGyro.getAngleY() <= 11) {
      m_RomiDrivetrain.arcadeDrive(0.5, 0);
    } else {
      isDetected = true;
    }

    if (isDetected) {
      m_RomiDrivetrain.arcadeDrive(m_PIDBalance.calculate(m_RomiDrivetrain.m_RomiGyro.getAngleY(), 0), m_PIDBalance.calculate(m_RomiDrivetrain.m_RomiGyro.getAngleZ(), 0));
    }
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
