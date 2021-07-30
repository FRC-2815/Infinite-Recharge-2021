// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax motor;
  private final DoubleSolenoid s;
  /** Creates a new Intake. */
  public Intake() {
    motor = new CANSparkMax(Constants.intakePort, MotorType.kBrushless);
    s = new DoubleSolenoid(0,4,5);
  }

  public void set(boolean on) {
    motor.set(on ? -.6: 0);
  }

  public void set(double p) {
    motor.set(p);
  }
  
  public void toggle() {
    s.toggle();
  }

  public void init() {
    s.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
