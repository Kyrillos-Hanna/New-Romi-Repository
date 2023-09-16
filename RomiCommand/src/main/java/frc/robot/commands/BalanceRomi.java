// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;

public class BalanceRomi extends CommandBase {

  private RomiDrivetrain m_RomiDrivetrain;
  private RomiGyro m_RomiGyro;
  private boolean alreadyTilted = false;
  private int angle;

  /** Creates a new BalanceRomi. */
  public BalanceRomi(RomiDrivetrain drivebase, RomiGyro gyroscope, int degrees) {
    m_RomiDrivetrain = drivebase;
    m_RomiGyro = gyroscope;
    angle = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_RomiDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_RomiDrivetrain.arcadeDrive(0,0);
    m_RomiGyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_RomiDrivetrain.arcadeDrive(0.5,0);

    if (m_RomiGyro.getAngleY() > Math.abs(angle)) {
      alreadyTilted = true;
      m_RomiDrivetrain.arcadeDrive(0.2, 0);
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
    if (m_RomiGyro.getAngleY() < 1 && m_RomiGyro.getAngleY() > -1) {
      return alreadyTilted;
    } else {
      return false;
    }
  }
}
