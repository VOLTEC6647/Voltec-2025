package com.team1678.frc2024.auto.actions;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Drive.DriveControlState;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.Superstructure;

import choreo.Choreo;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SwervePIDAction implements Action {
	private Drive mDrive = null;
	private Timer autoTimer = new Timer();
	private double correctionDelay = 1.5;
	Pose2d endpose;

	public SwervePIDAction(Pose2d endpose) {
		this.endpose = endpose;
	}

	@Override
	public void start() {
		System.out.println("PID Started");
		mDrive = Drive.getInstance();
		Superstructure s = Superstructure.getInstance();
		mDrive.setPIDSetpoint(new com.team254.lib.geometry.Pose2d(endpose));
	}

	@Override
	public void update() {
	}

	@Override
	public boolean isFinished() {
		return mDrive.isAtPIDSetpoint();
	}

	@Override
	public void done() {
		System.out.println("PID Finished");
		//mDrive.setOpenLoop(new ChassisSpeeds());
	}
}
