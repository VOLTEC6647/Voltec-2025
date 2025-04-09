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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GoCoralCommand extends SequentialCommandGroup {
  public GoCoralCommand() {
    Superstructure s = Superstructure.getInstance();
    final Pose2d endpose;
        if(s.level<4){
            endpose = s.getActiveCoral().toLegacy();
        }else{
            endpose = s.getActiveCoralL4().toLegacy();
        }
    addCommands(
      new DriveToPose(CommandSwerveDrivetrain.getInstance(), endpose)
    );
  }

}
