// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.Builder;
import org.littletonrobotics.frc2025.Constants6328;
import org.littletonrobotics.frc2025.Constants6328.RobotType;
import org.littletonrobotics.frc2025.util.swerve.ModuleLimits;

import com.team1678.frc2024.Ports1678;

public class DriveConstants {
  public static final double odometryFrequency = 250;
  public static final double trackWidthX =
      Constants6328.getRobot() == RobotType.DEVBOT
          ? Units.inchesToMeters(20.75)
          : Units.inchesToMeters(20.75);
  public static final double trackWidthY =
      Constants6328.getRobot() == RobotType.DEVBOT
          ? Units.inchesToMeters(20.75)
          : Units.inchesToMeters(20.75);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed = 4.69;
  public static final double maxAngularSpeed = 4.69 / driveBaseRadius;
  public static final double maxLinearAcceleration = 22.0;

  /** Includes bumpers! */
  public static final double robotWidth =
      Units.inchesToMeters(28.0) + 2 * Units.inchesToMeters(2.0);

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double wheelRadius = Units.inchesToMeters(2.000);

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(maxLinearSpeed, maxLinearAcceleration, Units.degreesToRadians(1080.0));

  public static final ModuleConfig[] moduleConfigsComp = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(Ports1678.FL_DRIVE.getDeviceNumber())
        .turnMotorId(Ports1678.FL_ROTATION.getDeviceNumber())
        .encoderChannel(Ports1678.FL_CANCODER.getDeviceNumber())
        .encoderOffset(Rotation2d.fromRotations(-0.437256+0.5).unaryMinus())
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(Ports1678.FR_DRIVE.getDeviceNumber())
        .turnMotorId(Ports1678.FR_ROTATION.getDeviceNumber())
        .encoderChannel(Ports1678.FR_CANCODER.getDeviceNumber())
        .encoderOffset(Rotation2d.fromRotations(-0.118408).unaryMinus())
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(Ports1678.BL_DRIVE.getDeviceNumber())
        .turnMotorId(Ports1678.BL_ROTATION.getDeviceNumber())
        .encoderChannel(Ports1678.BL_CANCODER.getDeviceNumber())
        .encoderOffset(Rotation2d.fromRotations(-0.354248+0.5).unaryMinus())
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(Ports1678.BR_DRIVE.getDeviceNumber())
        .turnMotorId(Ports1678.BR_ROTATION.getDeviceNumber())
        .encoderChannel(Ports1678.BR_CANCODER.getDeviceNumber())
        .encoderOffset(Rotation2d.fromRotations(-0.472656).unaryMinus())
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static final ModuleConfig[] moduleConfigsDev = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(12)
        .turnMotorId(9)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRadians(-0.009115335014721037))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(2)
        .turnMotorId(10)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRadians(0.8427416931125384))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(15)
        .turnMotorId(11)
        .encoderChannel(0)
        .encoderOffset(Rotation2d.fromRadians(-1.0620197413817225))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(8)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRadians(-2.600063124240756))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static class PigeonConstants {
    public static final int id = Constants6328.getRobot() == RobotType.DEVBOT ? Ports1678.PIGEON : Ports1678.PIGEON;
  }

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
