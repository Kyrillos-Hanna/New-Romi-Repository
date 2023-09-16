// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.RomiDrivetrain;

public class MoveWithController extends CommandBase {

  private RomiDrivetrain m_RomiDrivetrain;
  private XboxController m_XboxController;
  /** Creates a new MoveWithController. */
  public MoveWithController(RomiDrivetrain drivebase, XboxController controller) {
    m_RomiDrivetrain = drivebase;
    m_XboxController = controller;
    // Use addRequirements() here to declare subsystem dependencies.
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
    m_RomiDrivetrain.arcadeDrive(m_XboxController.getLeftY(), m_XboxController.getRightX());
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
