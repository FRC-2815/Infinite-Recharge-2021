// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MoveIntake extends CommandBase {
  private final Intake intake;
  private final BooleanSupplier button;
  private final DoubleSupplier axis;
  private final BooleanSupplier toggle;
  private final BooleanSupplier in;
  private final BooleanSupplier out;
  private final BooleanSupplier down;
  private final BooleanSupplier up;

  /** Creates a new MoveIntake. */
  public MoveIntake(Intake i, DoubleSupplier a, BooleanSupplier b, BooleanSupplier t, BooleanSupplier i2, BooleanSupplier o, BooleanSupplier d, BooleanSupplier u) {
    intake = i;
    axis = a;
    button = b;
    toggle = t;
    in = i2;
    out = o;
    down = d;
    up = u;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean pressed = button.getAsBoolean();
    if (pressed) {
      intake.set(.8);
    } else if (in.getAsBoolean()) {
        intake.set(.4);
    } else if (out.getAsBoolean()) {
        intake.set(-.4);
    } else {
    intake.set(axis.getAsDouble() * .4);
    }
    boolean t = toggle.getAsBoolean();
    if (t) {
      intake.toggle();
    }
    if (down.getAsBoolean()) {
      intake.setUp();
    }
    if (up.getAsBoolean()) {
      intake.setDown();
    }
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
