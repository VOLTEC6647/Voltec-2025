package com.team1678.frc2024.auto.actions;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.lib.swerve.ChassisSpeeds;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.vision.VisionSubsystem;

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

public class ResetGyroAction implements Action {

	private Drive mDrive = null;

	private Rotation2d rotation;
	private Pose2d pose;


	public ResetGyroAction(Rotation2d rotation) {
		mDrive = Drive.getInstance();
		this.rotation = rotation;
	}

	public ResetGyroAction() {
		mDrive = Drive.getInstance();
	}

	@Override
	public void start() {
		pose = VisionSubsystem.getInstance().getBestPose();
		//if(pose != null){
			this.rotation = pose.getRotation();
		//}
		//if(rotation!=null){
			Drive.getInstance().zeroGyro(rotation.getDegrees());
		//}
	}

	@Override
	public void update() {
		System.out.println("Trajectory set");
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void done() {
		mDrive.feedTeleopSetpoint(new ChassisSpeeds());
	}
}
