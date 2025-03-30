package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.GoCoralAction;
import com.team1678.frc2024.auto.actions.GoIntakeAction;

import com.team1678.frc2024.auto.actions.RequestAction;

import com.team1678.frc2024.auto.actions.SwervePIDAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForPrereqAction;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.lib.requests.DepositCoralRequest;
import com.team1678.lib.requests.IntakeCoralRequest;
import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class A6CPID extends AutoModeBase {
	private Drive d = Drive.getInstance();
	private Superstructure s = Superstructure.getInstance();
	private VisionSubsystem v = VisionSubsystem.getInstance();
	public A6CPID
	() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		//runAction(new ResetOdometryAction("5R1"));
		runAction(new WaitForPrereqAction(()->d.readyForAuto()));
		s.setLevel(4);
		s.coralId = 0;
		s.subCoralId = 1;

		//runAction(new ResetGyroAction());
		runAction(new GoCoralAction());
		runAction(new RequestAction(DepositCoralRequest.get()));
		CoralPivot.getInstance().setSetpointMotionMagic(CoralPivot.kIntakingAngle);

		System.out.println("Finished auto!");
	}
	// spotless:on
}