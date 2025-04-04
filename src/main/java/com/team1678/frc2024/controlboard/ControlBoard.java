package com.team1678.frc2024.controlboard;

import org.littletonrobotics.frc2025.subsystems.drive.Drive;

import com.team1678.frc2024.Constants1678;
import com.team1678.lib.Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
	private double kSwerveDeadband = Constants1678.stickDeadband;

	private static ControlBoard mInstance = null;

	private double globalMultiplyer = 1;

	public static ControlBoard getInstance() {
		if (mInstance == null) {
			mInstance = new ControlBoard();
		}

		return mInstance;
	}

	public final CustomXboxController1678 driver;
	public final CustomXboxController1678 operator;

	private ControlBoard() {
		driver = new CustomXboxController1678(0);
		operator = new CustomXboxController1678(Constants1678.kButtonGamepadPort);
	}

	public void update() {

		driver.update();
		operator.update();
	}

}
