// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team6647.frc2025.commands;

import org.littletonrobotics.frc2025.commands.DriveToPose;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;

import com.team4678.CommandSwerveDrivetrain;
import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GoIntakeCommand extends SequentialCommandGroup {
  public GoIntakeCommand() {
    Superstructure s = Superstructure.getInstance();
    CoralRoller.getInstance().setState(CoralRoller.State.CONSTANT);
    Pose2d endpose = s.sourcePose.toLegacy();
    
    addCommands(
      new DriveToPose(CommandSwerveDrivetrain.getInstance(), endpose)
    );
  }

}
