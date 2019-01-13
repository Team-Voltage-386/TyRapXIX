/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shifter extends Subsystem {
  public static final DoubleSolenoid.Value SLOW_GEAR = DoubleSolenoid.Value.kReverse;
  public static final DoubleSolenoid.Value FAST_GEAR = DoubleSolenoid.Value.kForward;

  DoubleSolenoid gearShifter = new DoubleSolenoid(RobotMap.gearShiftSolenoidForwardChannel,RobotMap.gearShiftSolenoidReverseChannel);

  /**
     * Shift gears. This method will shift to the opposite of the current gear.
     */
    public void shift() {
      if (gearShifter.get() == FAST_GEAR) {
          shift(SLOW_GEAR);
      } else {
          shift(FAST_GEAR);
      }
  }

  /**
     * Shift the gearShifter specifically to either low or high gear.
     * 
     * @param gear SLOW_GEAR or FAST_GEAR
     */
    public void shift(Value gear) {
      gearShifter.set(gear);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
