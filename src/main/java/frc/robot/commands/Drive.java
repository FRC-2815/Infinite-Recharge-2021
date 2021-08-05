/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class Drive extends CommandBase {
  private final DriveTrain driveTrain;
  
  private final DoubleSupplier forwardsAxis;
  private final DoubleSupplier turnAxis;
  private double v;
  private double x;
  private double y;
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private BooleanSupplier aim;
  private BooleanSupplier aim2;
  private final BooleanSupplier shoot;
  private final Shooter shooter;


  /**
   * Creates a new Drive.
   */
  public Drive(DriveTrain d, DoubleSupplier f, DoubleSupplier t, BooleanSupplier a, Shooter s, BooleanSupplier a2,
  BooleanSupplier s2) {
    driveTrain = d;
    forwardsAxis = f;
    turnAxis = t;
    aim = a;
    shooter = s;
    aim2 = a2;
    shoot = s2;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");
    ty = table.getEntry("ty");
  }

  private double turn;
  private double distance;
  private double rA; //relative angle
  private double kPAim = .08;
  private double kIAim;
  private double kDAim;
  private double maxRPM = 5200;
  private double forward;
  private double encoderTicks;
  private boolean shooting;
  

  @Override
  public void execute() {
    //read values periodically
    v = tv.getDouble(0.0);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("Target?", v);
    boolean a = aim.getAsBoolean() || aim2.getAsBoolean();
    SmartDashboard.putBoolean("Aim?", a);


    if (a && v == 1.0) {
      distance = (38.5 + 72) / Math.tan(7 + y);
      SmartDashboard.putNumber("Distance to target", distance);
      if (distance > 210) {
        forward = distance - 210;
        encoderTicks = (forward / Constants.wheelCircumference) * 4096;
        driveTrain.driveTicks(encoderTicks);
      }else if (distance > 0) {
        shooter.set((distance/210) * maxRPM);
      }
      rA = (x / 35);
      if (Math.abs(x) >= 1.0) {
        turn =  rA + rA * kPAim;
      }
    }else {
      if (shooting) {
        shooter.set(5200);
      }else {
        shooter.set(0);
      }
      turn = 0;
    }
      boolean toggleShoot = shoot.getAsBoolean();
      SmartDashboard.putBoolean("Shooter on", shooting);
    if (toggleShoot) {
      shooting = !shooting;
    }
    
    double forwardsValue = -forwardsAxis.getAsDouble();
    double turnValue = turnAxis.getAsDouble() * .8;

    if (Math.abs(forwardsValue) < .05) {
      forwardsValue = 0.0;
    }

    if (Math.abs(turnValue) < .05) {
      turnValue = 0.0;
    }

    double forwardsNegation = 1.0;
    double turnNegation = 1.0;

    if (forwardsValue < 0.0) {
      forwardsNegation = -1.0;
    }

    if (turnValue < 0.0) {
      turnNegation = -1.0;
    }

    driveTrain.drive(Math.pow(Math.abs(forwardsValue), 1.5) * forwardsNegation, Math.pow(Math.abs(turnValue), 1.5) * turnNegation + turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
