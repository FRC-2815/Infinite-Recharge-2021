/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonSRX[] talons = new WPI_TalonSRX[4];
  private final DifferentialDrive botDrive;

  

  
  public DriveTrain() { // Creates a new DriveTrain.
    for (int i = 0; i < talons.length; i++) { // for talons in talons.length
      talons[i] = new WPI_TalonSRX(Constants.driveTalonPorts[i]); // talons[0, 1, 2, 3] = new talons[0, 1, 2, 3]
    }

    configNormal();

    botDrive = new DifferentialDrive(talons[Constants.mainTalonPorts[0]], talons[Constants.mainTalonPorts[1]]); // defining differential drive mode with main talons
    botDrive.setRightSideInverted(true); // sets right side motors inverted
  }

  public void configNormal() { // normal configuration function
    for (WPI_TalonSRX talon : talons) { // for talon in talons
      talon.configFactoryDefault(); // configure factory default
      talon.setSensorPhase(true); // sets phase
      talon.setSelectedSensorPosition(0); // sets sensor position = 0
    }

    for (int i = 0; i < 2; i++) { // for  2 talons in talons
      talons[Constants.mainTalonPorts[i]].setInverted(false); // sets left motors to be non inverted
      
      talons[Constants.bareTalonPorts[i]].follow(talons[Constants.mainTalonPorts[i]]); // talons with no encoder follows talons with encoder
      talons[Constants.bareTalonPorts[i]].setInverted(InvertType.FollowMaster); // sets right talon inverted
    }
  }

  public void configPositionDrive() { // configPositionDrive function
    for (WPI_TalonSRX talon : talons) { // for talon in talons
      talon.configFactoryDefault(); // reset to factory default
      talon.setSensorPhase(true); // set phase
      talon.setSelectedSensorPosition(0); // set sensor position = 0

      talon.configNominalOutputForward(0.0); // sets nominal output forward speed?
      talon.configNominalOutputReverse(0.0); // sets nominal output reverse speed?
      talon.configPeakOutputForward(1.0); // sets desired peak output speed?
      talon.configPeakOutputReverse(1.0); // sets desired peak output reverse speed?

      talon.configAllowableClosedloopError(0, 0.0); // ???

      talon.config_kF(0, 0.0); // sets F constant in PID loop
      talon.config_kP(0, 0.15); // sets P constant in PID loop
      talon.config_kI(0, 0.0); // sets I constant in PID loop
      talon.config_kD(0, 1.0); // sets D constant in PID loop
    }

    talons[Constants.mainTalonPorts[1]].setInverted(true); // sets talon[1] inverted

    for (int i = 0; i < 2; i++) { // for 2 talons in talons
      talons[Constants.bareTalonPorts[i]].follow(talons[Constants.mainTalonPorts[i]]); // bare talons follow main talons
      talons[Constants.bareTalonPorts[i]].setInverted(InvertType.FollowMaster); // set right bare talon inverted
    }
  }

  public void driveTicks(double ticks) { // driveTicks function
    talons[Constants.mainTalonPorts[0]].set(ControlMode.Position, ticks); // sets maintalon[0] (talon[0]) to proper given ticks
    talons[Constants.mainTalonPorts[1]].set(ControlMode.Position, ticks); // sets maintalon[1] (talon[3]) to proper given ticks
  }

  public void drive(double f, double t) { // drive function
    SmartDashboard.putNumber("forw", f); // output f variable to smartdashboard as "forw"
    SmartDashboard.putNumber("turn", t); // output t variable to smartdashboard as "turn"

    double ePLeft = talons[Constants.mainTalonPorts[0]].getSelectedSensorPosition(); // get maintalonport[0] sensor position
    double ePRight = talons[Constants.mainTalonPorts[1]].getSelectedSensorPosition();// get maintalonport[1] sensor position

    SmartDashboard.putNumber("ePLeft", ePLeft); // output eLeft variable to smartdashboard as "eLeft"
    SmartDashboard.putNumber("ePRight", ePRight); // output eRight variable to smartdashboard as "eRight"

    double eVLeft = talons[Constants.mainTalonPorts[0]].getSelectedSensorVelocity(); // get maintalonport[0] sensor velocity
    double eVRight = talons[Constants.mainTalonPorts[1]].getSelectedSensorVelocity(); // get maintalonport[1] sensor velocity

    if (t == 0) { // if t variable = 0
      t = (eVLeft + eVRight) / 768 * -Math.abs(f); // t variable = sum(velocityErrors) / 768 * negative absolutevalue of f variable
      SmartDashboard.putNumber("tError", t); // output t variable to smartdashboard as "tError"
    }
    
    botDrive.curvatureDrive(f, t, true); // Enable curvature drive
  }

  public double getAvgPower() { // getavgpower function
    double power = 0; // power = 0

    for (WPI_TalonSRX talon : talons) { // for talon in talons
      power += talon.get(); // power = power + current talons speed
    }

    return power / talons.length; // returns power / number of talons
  }

  public void resetEncoders() { // resetencoder function
    talons[Constants.mainTalonPorts[0]].setSelectedSensorPosition(0); // set encoder position on talon[0] to 0
    talons[Constants.mainTalonPorts[1]].setSelectedSensorPosition(0); // set encoder position on talon[1] to 0
  }

  public double[] getEncoders() { // getencoders function
    return new double[] { // returns new double
      talons[Constants.mainTalonPorts[0]].getSelectedSensorPosition(), // retrieves sensor position from encoder on maintalonport[0]
      talons[Constants.mainTalonPorts[1]].getSelectedSensorPosition() // retrieves sensor position from encoder on maintalonport[1]
    };
  }

  public double[] getEncodersVelocity() { // getencodersvelocity function
    return new double[] { // returns new double
      talons[Constants.mainTalonPorts[0]].getSelectedSensorVelocity(), // retrieves sensor velocity from encoder on maintalonport[0]
      talons[Constants.mainTalonPorts[1]].getSelectedSensorVelocity(), // retrieves sensor velocity from encoder on maintalonport[1]
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}