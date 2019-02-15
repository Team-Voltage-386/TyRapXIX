/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SpikeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Relay lightRelay = new Relay(RobotMap.spikeLightRing);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public SpikeSubsystem() {
    lightRelay.set(Relay.Value.kForward);
  }

  public void spikeToggle() {
    if (lightRelay.get() == Relay.Value.kOff) {
      lightRelay.set(Relay.Value.kForward);
    } else if (lightRelay.get() == Relay.Value.kForward) {
      lightRelay.set(Relay.Value.kOff);
    }
  }

  public void lightSwitch() {
    lightRelay.set(Relay.Value.kForward);
  }

  public Relay.Value getState() {
    return lightRelay.get();
  }
}