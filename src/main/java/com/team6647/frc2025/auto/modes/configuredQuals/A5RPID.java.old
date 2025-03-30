package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.GoCoralAction;
import com.team1678.frc2024.auto.actions.GoIntakeAction;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.ParallelAction;
import com.team1678.frc2024.auto.actions.PathplannerPathAction;
import com.team1678.frc2024.auto.actions.PathplannerTrajectoryAction;
import com.team1678.frc2024.auto.actions.RequestAction;
import com.team1678.frc2024.auto.actions.ResetGyroAction;
import com.team1678.frc2024.auto.actions.ResetOdometryAction;
import com.team1678.frc2024.auto.actions.SwervePIDAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForPrereqAction;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Drive.DriveControlState;
import com.team1678.lib.requests.DepositCoralRequest;
import com.team1678.lib.requests.IntakeCoralRequest;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.SequentialRequest;
import com.team1678.lib.requests.WaitForPrereqRequest;
import com.team1678.lib.requests.WaitRequest;
import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;
import com.team6647.frc2025.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class A5RPID extends AutoModeBase {
	private Drive d = Drive.getInstance();
	private Superstructure s = Superstructure.getInstance();
	private VisionSubsystem v = VisionSubsystem.getInstance();
	public A5RPID() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		//runAction(new ResetOdometryAction("5R1"));
		
		runAction(new WaitForPrereqAction(()->d.readyForAuto()));
		s.setLevel(2);
		s.coralId = 5;
		s.subCoralId = 1;
		runAction(new SwervePIDAction(FieldLayout.handleAllianceFlip(new Pose2d(5.392,5.585,Rotation2d.fromDegrees(60)), Robot.is_red_alliance)));

		//runAction(new ResetGyroAction());
		runAction(new GoCoralAction());
		runAction(new RequestAction(DepositCoralRequest.get()));
		CoralPivot.getInstance().setSetpointMotionMagic(CoralPivot.kIntakingAngle);
		runAction(new SwervePIDAction(FieldLayout.handleAllianceFlip(new Pose2d(5.203,6.153,Rotation2d.fromDegrees(0)), Robot.is_red_alliance)));
		s.setLevel(2);
		s.coralId = 4;
		s.subCoralId = 1;
		runAction(new SwervePIDAction(FieldLayout.handleAllianceFlip(new Pose2d(1.872,6.550,Rotation2d.fromDegrees(-55)), Robot.is_red_alliance)));

		runAction(new GoIntakeAction());
		runAction(new RequestAction(IntakeCoralRequest.get()));

		runAction(new WaitAction(0.2));
		runAction(new GoCoralAction());
		runAction(new RequestAction(DepositCoralRequest.get()));


		System.out.println("Finished auto!");
	}
	// spotless:on
}