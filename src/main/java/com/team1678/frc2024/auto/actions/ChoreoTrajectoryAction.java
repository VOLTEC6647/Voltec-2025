package com.team1678.frc2024.auto.actions;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import com.team6647.frc2025.Robot;
import choreo.Choreo;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

public class ChoreoTrajectoryAction implements Action {

	private Drive mDrive = null;
	private RobotState state = null;

	Optional<Trajectory<SwerveSample>> trajectory;
	private final boolean mResetGyro;

	private Timer autoTimer = new Timer();
	private List<EventMarker> events;
	private double correctionDelay = 1.5;
				

	public ChoreoTrajectoryAction(String trajectory) {
		this(trajectory, false);
	}

	public ChoreoTrajectoryAction(String trajectoryName, boolean resetPose) {
		trajectory = Choreo.loadTrajectory(trajectoryName);
		mDrive = Drive.getInstance();
		state = RobotState.getInstance();
		mResetGyro = resetPose;
	}

	public ChoreoTrajectoryAction(String trajectoryName, boolean resetPose, double correctionTime) {
		trajectory = Choreo.loadTrajectory(trajectoryName);
		mDrive = Drive.getInstance();
		mResetGyro = resetPose;
		this.correctionDelay = correctionTime;
	}

	@Override
	public void start() {
		mDrive.acceptingHeading = 1;
		//events = trajectory.get().events();
		autoTimer.reset();
		if (mResetGyro) {
			state.resetPose(trajectory.get().getInitialPose(Robot.is_red_alliance).get());
			System.out.println("Reset odometry to " + state.getRotation().getDegrees());
		}
		autoTimer.start();		
	}

	@Override
	public void update() {
		Optional<SwerveSample> sample = trajectory.get().sampleAt(autoTimer.get(), Robot.is_red_alliance);
		if (sample.isPresent()) {
			mDrive.choreoController(sample.get());
			//Logger.recordOutput("serrrr",sample.get()); crash
		}else{
			System.out.println("Sample not present");
		}
		if(trajectory.get().getTotalTime() < autoTimer.get()){
			mDrive.acceptingHeading = 1;
		}
		/*
		events.forEach(event -> {
			if (event.timestamp < autoTimer.get()) {
				if(event.event.equals("Elevate")){
					Superstructure.getInstance().prepareLevel(Superstructure.getInstance().currentLevel);
					events.remove(event);
				}
			}
		});
		*/
		System.out.println("Trajectory set");
	}

	@Override
	public boolean isFinished() {
		return trajectory.get().getTotalTime()+correctionDelay < autoTimer.get();
	}

	@Override
	public void done() {
		mDrive.stop();
	}
}
