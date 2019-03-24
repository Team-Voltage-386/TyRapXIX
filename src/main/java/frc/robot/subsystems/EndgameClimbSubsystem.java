// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.PowerDistributionPanel;
// import edu.wpi.first.wpilibj.Spark;
// import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.RobotMap;
// import frc.robot.commands.climb.ManualClimb;

// /**
// * Add your docs here.
// */
// public class EndgameClimbSubsystem extends Subsystem {
// // CURRENT LIMITS ON ELEVATOR TALON
// private Spark leftClimbArm = new Spark(RobotMap.leftClimbArm);
// private Spark rightClimbArm = new Spark(RobotMap.rightClimbArm);
// private Spark climbElevatorWheels = new Spark(RobotMap.elevatorDriveWheels);
// private WPI_TalonSRX climbElevatorMotor = new
// WPI_TalonSRX(RobotMap.rearElevatorMotor);
// private DigitalInput elevatorLimitSwitch = new
// DigitalInput(RobotMap.elevatorLimitSwitch);

// private PowerDistributionPanel pdp = new PowerDistributionPanel();

// @Override
// public void initDefaultCommand() {
// setDefaultCommand(new ManualClimb());
// }

// public void setClimbArmSpeeds(double speed) {
// leftClimbArm.set(-1 * speed);
// rightClimbArm.set(speed);
// }

// public void setElevatorSpeed(double speed) {
// climbElevatorMotor.set(speed);
// }

// public void setElevatorWheelsSpeed(double speed) {
// climbElevatorWheels.set(speed);
// }

// public boolean getElevatorLimitSwitch() {
// return elevatorLimitSwitch.get();
// }

// /** ElevatorLimit, Arm PDP (4 and 11) */
// public void displayDiagnostics() {
// SmartDashboard.putBoolean("ElevatorLimitSwitch", getElevatorLimitSwitch());
// SmartDashboard.putNumber("PDP 4 Current", getPDPCurrent(4));
// SmartDashboard.putNumber("PDP 11 Current", getPDPCurrent(11));
// }

// public double getPDPCurrent(int pdpPort) {
// return pdp.getCurrent(pdpPort);
// }

// }
