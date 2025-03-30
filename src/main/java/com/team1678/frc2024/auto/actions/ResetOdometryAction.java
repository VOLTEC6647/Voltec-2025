package com.team1678.frc2024.auto.actions;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.frc2025.RobotState6328;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import com.team1678.frc2024.RobotState;
import com.team1678.lib.swerve.ChassisSpeeds;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ResetOdometryAction implements Action {

	private final String trajectory;

	private Timer autoTimer = new Timer();
	private double correctionDelay = 1.5;
	private Command pathPlannerCommand;
	private PathPlannerPath pathPlannerPath;


	public ResetOdometryAction(String trajectory) {
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
	}

	@Override
	public void start() {
		RobotState6328.getInstance().resetPose(pathPlannerPath.getStartingHolonomicPose().get());
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
		
	}
}
