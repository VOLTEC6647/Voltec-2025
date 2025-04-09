// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveToPose extends Command {
  private final Drive m_swerve;
  private final Pose2d m_targetPose;
  
  // PID controllers for X, Y, and Rotation
  private final PhoenixPIDController m_xController = new PhoenixPIDController(0.04, 0, 0.001);
  private final PhoenixPIDController m_yController = new PhoenixPIDController(0.036, 0, 0.002);
  private final PhoenixPIDController m_rotationController = new PhoenixPIDController(7.1, 0, 0.15);
  
  // Motion values
  private double m_xSpeed = 0.0;
  private double m_ySpeed = 0.0;
  private double m_rotationSpeed = 0.0;
  
  // Alignment state tracking
  private boolean m_isDone = false;
  private final Debouncer m_alignmentDone = new Debouncer(0.1);
  
  // Speed limits
  private final double m_maxSpeed;
  private final double m_maxAngularRate;
  
  // Drive requests
  private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Creates a command that aligns the robot to a specified Pose2d.
   * 
   * @param swerve The swerve drive subsystem
   * @param targetPose The target pose to align to
   * @param maxSpeed Maximum linear speed in meters per second
   * @param maxAngularRate Maximum angular rate in rotations per second
   */
  public DriveToPose(Drive swerve, Pose2d targetPose, double maxSpeed, double maxAngularRate) {
    m_swerve = swerve;
    m_targetPose = targetPose;
    m_maxSpeed = maxSpeed;
    m_maxAngularRate = RotationsPerSecond.of(maxAngularRate).in(RadiansPerSecond);
    
    // Configure the rotation controller for continuous input
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Set tolerances for alignment
    m_xController.setTolerance(2.5); // 2.5 cm tolerance
    m_yController.setTolerance(2.5); // 2.5 cm tolerance
    m_rotationController.setTolerance(Math.toRadians(2.5)); // 2.5 degrees tolerance
    
    addRequirements(swerve);
  }

  /**
   * Creates a command that aligns the robot to a specified Pose2d with default speed limits.
   * 
   * @param swerve The swerve drive subsystem
   * @param targetPose The target pose to align to
   */
  public DriveToPose(Drive swerve, Pose2d targetPose) {
    this(swerve, targetPose, MetersPerSecond.of(4.572).magnitude(), 0.75); // Default to 2.5 m/s max speed and 0.75 rotations per second
  }

  @Override
  public void initialize() {
    m_isDone = false;
    m_alignmentDone.calculate(false);
    
    // Reset PID controllers
    m_xController.reset();
    m_yController.reset();
    m_rotationController.reset();
    
    // Initialize PID values with adaptive gains
    m_xController.setP(0.041);
    m_yController.setP(0.048);
    
    // Signal that we're auto aligning
    m_swerve.setIsAutoAligning(true);
  }

  @Override
  public void execute() {
    // Get current pose
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    
    // Calculate position errors
    double xError = m_targetPose.getX() * 100 - currentPose.getX() * 100; // Convert to cm for PID
    double yError = m_targetPose.getY() * 100 - currentPose.getY() * 100; // Convert to cm for PID
    double rotationError = m_targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
    
    // Adjust PID gains based on error magnitude (fine tuning when close)
    if (Math.abs(xError) < 8) {
      m_xController.setP(0.057);
    } else {
      m_xController.setP(0.041);
    }
    
    if (Math.abs(yError) < 10) {
      m_yController.setP(0.057);
    } else {
      m_yController.setP(0.048);
    }
    
    // Calculate control outputs
    m_xSpeed = m_xController.calculate(currentPose.getX() * 100, m_targetPose.getX() * 100, Timer.getFPGATimestamp());
    m_ySpeed = m_yController.calculate(currentPose.getY() * 100, m_targetPose.getY() * 100, Timer.getFPGATimestamp());
    m_rotationSpeed = m_rotationController.calculate(
        currentPose.getRotation().getRadians(), 
        m_targetPose.getRotation().getRadians(), 
        Timer.getFPGATimestamp());
    
    // Add feedforward to improve response
    if (m_xSpeed < 0) {
      m_xSpeed -= 0.08;
    } else {
      m_xSpeed += 0.08;
    }
    
    if (m_ySpeed < 0) {
      m_ySpeed -= 0.1;
    } else {
      m_ySpeed += 0.1;
    }
    
    // Clamp speeds to maximum values
    m_xSpeed = Math.max(-m_maxSpeed, Math.min(m_maxSpeed, m_xSpeed));
    m_ySpeed = Math.max(-m_maxSpeed, Math.min(m_maxSpeed, m_ySpeed));
    m_rotationSpeed = Math.max(-m_maxAngularRate, Math.min(m_maxAngularRate, m_rotationSpeed));
    
    // Log values to SmartDashboard
    SmartDashboard.putNumber("AutoAlign/X Speed", m_xSpeed);
    SmartDashboard.putNumber("AutoAlign/Y Speed", m_ySpeed);
    SmartDashboard.putNumber("AutoAlign/Rotation Speed", m_rotationSpeed);
    SmartDashboard.putNumber("AutoAlign/X Error", xError);
    SmartDashboard.putNumber("AutoAlign/Y Error", yError);
    SmartDashboard.putNumber("AutoAlign/Rotation Error", rotationError);
    SmartDashboard.putBoolean("AutoAlign/X At Setpoint", m_xController.atSetpoint());
    SmartDashboard.putBoolean("AutoAlign/Y At Setpoint", m_yController.atSetpoint());
    SmartDashboard.putBoolean("AutoAlign/Rotation At Setpoint", m_rotationController.atSetpoint());
    
    m_swerve.set
    // Apply control to swerve drive
    m_swerve.setControl(
      m_driveRequest.withVelocityX(-m_ySpeed)
          .withVelocityY(m_xSpeed)
          .withRotationalRate(m_rotationSpeed)
    );
    
    // Check if alignment is complete
    m_isDone = m_alignmentDone.calculate(
      m_xController.atSetpoint() && 
      m_yController.atSetpoint() && 
      m_rotationController.atSetpoint()
    );
    
    SmartDashboard.putBoolean("AutoAlign/IsDone", m_isDone);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.setIsAutoAligning(false);
  }

  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}