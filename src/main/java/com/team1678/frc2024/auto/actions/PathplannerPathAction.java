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
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.Superstructure;

import choreo.Choreo;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PathplannerPathAction implements Action {


	private final String trajectory;

	private Timer autoTimer = new Timer();
	private double pathTime;
	private double correctionDelay = 1.5;
	private Command pathPlannerCommand;
	private PathPlannerPath pathPlannerPath;


	public PathplannerPathAction(String trajectory) {
		this.trajectory = trajectory;

		try {
			pathPlannerPath = PathPlannerPath.fromPathFile(trajectory);
		} catch (FileVersionException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (ParseException e) {
			e.printStackTrace();
		}
		pathPlannerCommand = AutoBuilder.followPath(pathPlannerPath);
		
		pathTime = 5;
		
	}

	@Override
	public void start() {
		autoTimer.reset();
		autoTimer.start();
		
		CommandScheduler.getInstance().schedule(pathPlannerCommand);
	}

	@Override
	public void update() {
		System.out.println("Trajectory set");
	}

	@Override
	public boolean isFinished() {
		return pathPlannerCommand.isFinished();
	}

	@Override
	public void done() {
		//mDrive.feedTeleopSetpoint(new ChassisSpeeds());
	}
}
