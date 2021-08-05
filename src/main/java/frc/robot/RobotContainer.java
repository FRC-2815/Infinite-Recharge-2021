/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.autoCommandGroups.TestAuto;
import frc.robot.subsystems.*;

/**
 * Le robot container
 */
public class RobotContainer {
  // Controllers
  private XboxController mano;
  private XboxController mano2;
  private Joystick stick;
  // private final XboxController mano2 =
  // DriverStation.getInstance().getJoystickName(1).isEmpty() ? mano : new
  // XboxController(1);

  // Important stuff that isn't a controller, command, or subsytem
  private final String driveMode = "normal";
  public final static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Tower tower = new Tower();
  private final Hopper hopper = new Hopper();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();

  // Commands
  private Drive drive;
  private MoveTower moveTower;
  private MoveHopper moveHopper;
  private ShootShooter shootShooter;
  private MoveIntake moveIntake;
  private Climb climb;
  private Aim aim;

  public RobotContainer() {
    setup();
  }

  public void setup() {
    mano = new XboxController(0);
    stick = new Joystick(3);
    if (DriverStation.getInstance().getJoystickName(1).isEmpty()) {
      mano2 = mano;
    } else {
      mano2 = new XboxController(1);
    }

    if (driveMode.equals("trigger")) {
      drive = new Drive(driveTrain,
          () -> (mano.getTriggerAxis(GenericHID.Hand.kRight) - mano.getTriggerAxis(GenericHID.Hand.kLeft)),
          () -> mano.getX(GenericHID.Hand.kRight), () -> mano2.getRawButton(9), shooter, () -> mano.getAButton(), () -> stick.getRawButtonPressed(2));
    } else {
      drive = new Drive(driveTrain, () -> mano.getY(GenericHID.Hand.kLeft), () -> mano.getX(GenericHID.Hand.kRight), () -> stick.getRawButton(9), shooter, () -> mano2.getAButton(),
      () -> stick.getRawButtonPressed(2));
    }

    moveTower = new MoveTower(tower, () -> mano2.getXButton(),
        () -> stick.getRawButton(1), () -> stick.getRawButton(6));

    moveHopper = new MoveHopper(hopper, () -> mano2.getXButton(), () -> stick.getRawButton(1), () -> stick.getRawButton(6));
    shootShooter = new ShootShooter(shooter, () -> mano2.getYButtonPressed(), () -> mano2.getPOV()/*, () -> stick.getRawButtonPressed(2)*/);

    moveIntake = new MoveIntake(intake,
          () -> (mano.getTriggerAxis(GenericHID.Hand.kRight) - mano.getTriggerAxis(GenericHID.Hand.kLeft)),
           () -> mano.getBButton(), () -> stick.getRawButtonPressed(5), () -> stick.getRawButton(3),
            () -> stick.getRawButton(4), () -> mano.getBumper(GenericHID.Hand.kRight), () -> mano.getBumper(GenericHID.Hand.kLeft));

    climb = new Climb(climber, () -> stick.getRawButtonPressed(11), () -> stick.getRawButtonPressed(12));

    aim = new Aim(stick.getRawButton(9));
    
    driveTrain.setDefaultCommand(drive);
    tower.setDefaultCommand(moveTower);
    hopper.setDefaultCommand(moveHopper);
    shooter.setDefaultCommand(shootShooter);
    intake.setDefaultCommand(moveIntake);
    climber.setDefaultCommand(climb);

    driveTrain.resetEncoders();
  }

  public static ADXRS450_Gyro getGyro() {
    return gyro;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public SequentialCommandGroup getAutonomousCommand() {
    return new TestAuto(driveTrain, shooter, hopper, tower);
  }
}