// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;



import frc.robot.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

    public static final class Swerve{

    
    public static final int kDriverControllerPort = 0;

    //PID constants & feedforward constants for swerve motors
    public static final double KP_value_drive = 0.0;
    public static final double KF_value_drive = 0.0;

    public static final double KP_value_steer = 0.0;
    public static final double KF_value_steer = 0.0;

    //REMEMBER angle of motor and swerve are different 

    public static double gear_ratio_coefficient;

    public static double maxSpeed;


    public static final class Mod0 {
      public static final int driveMotorID;
      public static final int angleMotorID;
      public static final double angleOffset;
      public static final int canCoderID;
      

      
      public static final SwerveModuleConstants constants = 
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }
  
  /* Front Right Module - Module 1 */
  public static final class Mod1 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 3;
      public static final double angleOffset = 6;
      public static final int mod_number1 = 1;public static final SwerveModuleConstants constants = 
      new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    
  }
  
  /* Back Left Module - Module 2 */
  public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12;
      public static final double angleOffset = 40.07811;
      public static final int mod_number2 = 2;
      public static final SwerveModuleConstants constants = 
      new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      
  }
  /* Back Right Module - Module 3 */
  public static final class Mod3 {
      public static final int driveMotorID = 9;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 11;
      public static final double angleOffset = -38.05664;
    
      public static final SwerveModuleConstants constants = 
      new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      

  
  }

  //parameters of drivetrain

  public static final double trackWidth;
  public static final double wheelBase;
  public static final double wheelDiameter;
  public static final double wheelCircumference;

  public static final double openLoopRamp;
  public static final double closedLoopRamp;

  public static final double driveGearRatio;
  public static final double angleGearRatio;

  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0), //FR
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), //BR
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), //FL
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); //BL
}
}
}

