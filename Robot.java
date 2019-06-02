/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1629.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// For the LIDAR
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

// This is an ENUM that is used to save "side of field" information
enum ActiveSide {
	LEFT, CENTER, RIGHT
}

enum AutoDriveMode {
	STOPPED, DRIVING, DRIVING_BRAKE, TURNING, SWEEPING
}

enum AutoMode {
	DO_NOTHING,
	CROSS_LINE, 
	SIMPLE_SWITCH, 
	SIMPLE_SWITCH_PLUS, 
	CENTER_PLAY, 
	CENTER_PLAY_PLUS, 
	SCALE_SWITCH, 
	SWITCH_SCALE, 
	CROSSOVER_SWITCH, 
	CROSSOVER_SCALE,
	LONG_RUN
}

enum YesNo {
	NO, YES
}

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {
	/*
	 * CHANGELOG 
	 * v1.1.0 added multiple autonomous modes 
	 * v1.1.1 INTEGRATED PREVIOUS * CODE FOR USE WITH THE COMPETITION ROBOT 
	 * 		  -fixed solenoid channels for * competition robot 
	 * 		  -added shifter control to lift gear-box on competition * robot 
	 * 		  -changed shiftS and shift button names to driveShiftS for clarification * given the new liftShiftS functionality 
	 * 		  -added buttons to control lift shifter *on pilot joy-stick 
	 * v1.1.2 -calibrated lift control to new string pot 
	 * 		  -fixed * limit switches to new logic -added variable gain to lift control based on * what gear the lift is in 
	 * v1.1.3 -changed co-pilot controls for button pad
	 * v1.2.0 - Added Navx IMU 
	 * v1.2.1 - Added "turn to heading" code 
	 * V1.2.2 - added * Switch-Bridge auto. 
	 * v1.3.0 - Improved move timeout & Updated Autos 
	 * V1.3.1 - * Slow Push 
	 * V1.3.2 - Faster Push, extend Center Play 
	 * V1.4.0 - Cross Over Auto
	 * V1.5.0 - added code for limelight
	 * V2.0.0 - Final build before ship 3/22/2018 1:00
	 * V2.0.1 - Not so close to scale
	 * V2.0.2 - Slower center second cube
	 * V2.0.3 - Shoots second cube on crossover, move farther forward, and crossover less
	 * V2.0.4 - Takes snapshot of cube in camera  Final version for MD Qualifiers
	 * V2.1.0 - Chesapeake Division Championship.  
	 * 	        Added autoCrossoverScale
	 *          Cleaned up limelight snapshot.  Now in auto tracking and manual track
	 * V2.1.1 - Slow down shooter
	 * V3.0.0 - Adding LIDAR
	 * V3.1.0 - Added easy string pot calibration
	 * 	 */
	// Constants
	final String VERSION = "3.1.0";

	// Take voltage measurements at the POT for the lift at the bottom, and raised 60 inches
	final double VOLTAGE_AT_0_INCHES  = 0.022;
	final double VOLTAGE_AT_60_INCHES = 2.928;
	
	// final double LIFT_SLOPE = 21.081;
	// final double LIFT_OFFSET = -0.054 / LIFT_SLOPE;
	
	final double LIFT_SLOPE  = (60.0 / (VOLTAGE_AT_60_INCHES - VOLTAGE_AT_0_INCHES));
	final double LIFT_OFFSET = (-VOLTAGE_AT_0_INCHES / LIFT_SLOPE );
	final double MAX_HEIGHT = 86;

	final double LIFT_GAIN_HIGH 		= 0.225;   // 0.15
	final double LIFT_GAIN_LOW 			= 0.30;    // 0.20
	final double LIFT_JOG_INCHES		= 0.3;
	final double LIFT_POS_FLOOR 		= 0.00;
	final double LIFT_POS_DRIVE 		= 5.0;
	final double LIFT_POS_STACK 		= 12.0;
	final double LIFT_POS_SWITCH 		= 24.0;
	final double LIFT_POS_AUTO_SWITCH 	= 30.0;
	final double LIFT_POS_SCALE_LOW 	= 54.0;
	final double LIFT_POS_SCALE_MID 	= 66.0;
	final double LIFT_POS_SCALE_HIGH 	= 78.0;
	final double LIFT_POS_CLIMB_LOWER 	= 55.0;
	final double LIFT_POS_CLIMB_RAISE 	= 79.0;

	final double COLLECT_SPEED = 0.75;
	final double EJECT_SPEED = -0.75;
	final double ROTATION_SPEED = -0.5;
	final double SLOW_PUSH = -0.4;

	final double GRAVITY_HOLD = 0.175;
	final double GRAVITY_LIFT = 0.3;

	final double HEADING_MATCH = 1.0;
	final double HEADING_TURN_GAIN = 0.05; // Full turn for greater then 20 deg error
	final double HEADING_DRIVE_GAIN = 0.015; // Full turn for greater then 60 deg error
	final double TRACKNG_TURN_POWER = 0.5; // Max poser when turning to cube
	final double DISTANCE_GAIN = 0.05; // Approaching the cube
	final double TRACKING_DRIVE_POWER = 0.6; // max power when approaching cube
	final double MAX_TURN_RATE = 0.75;
	final double MAX_STOPPED_DISTANCE = 0.1; // Must move less inches in a cycle to be considered stopped.
	final double BRAKE_DISTANCE = 24.0; // Start decelerating at this distance from full speed.
	final double DRIVE_BRAKE_GAIN = 1.0 / BRAKE_DISTANCE;
	final boolean BRAKE = true;
	final boolean COAST = false;

	final double WIDTH_OF_MOUTH = 5.0;
	final double DEPTH_OF_MOUTH = 3.0;
	final double DISTANT_CUBE = 20.0;

	// Choose-able Selections
	private SendableChooser<Integer> autoDelay = new SendableChooser<>();
	private SendableChooser<ActiveSide> startPosition = new SendableChooser<>();
	private SendableChooser<AutoMode> autoMode = new SendableChooser<>();
	private SendableChooser<YesNo> crossOver = new SendableChooser<>();

	private Timer autoTimer;
	private Timer timer;
	private AHRS gyro;
	private Counter lidar;


	// Limelight tracking information
	NetworkTable table;
	NetworkTableEntry tx;
	NetworkTableEntry ty;
	NetworkTableEntry ta;
	NetworkTableEntry tv;
	NetworkTableEntry led;
	NetworkTableEntry snapshot;
	boolean target_valid;
	double target_x;
	double target_y;
	double target_area;
	String gameData = new String("none");

	// Basic Instance Variables
	double liftHeight = 0;
	double curLiftGain = 0;
	double liftSetPoint = 0;
	boolean liftInPosition = false;
	boolean goingToFloor = false;
	boolean getExtraCube = false;
	boolean robotIsMoving = false;
	boolean trackingCube = false;
	boolean holdingCube = false;
	boolean grabbingCube = false;
	boolean cubeInMouth = false;
	boolean takeSnapshots = true;

	double filteredDrive = 0;
	double filteredTurn = 0;
	double robotHeading = 0;
	double lastDistanceTraveled = 0;
	double inchesToWall = 0;

	// Autonomous Drive Variables
	AutoDriveMode driveMode = AutoDriveMode.STOPPED;
	double distanceTarget = 0;
	boolean distanceCorrect = true;
	double headingTarget = 0;
	boolean headingCorrect = true;
	double drivePower = 0;
	double timeout = 0;
	double headingError = 0;

	int curAutoState = 0;
	int selAutoDelay = 0;
	int POV = 0;

	AutoMode selAutoMode = AutoMode.DO_NOTHING;
	YesNo selCrossOver = YesNo.NO;

	ActiveSide selStartPosition = ActiveSide.LEFT;
	ActiveSide switchPosition = ActiveSide.CENTER;
	ActiveSide scalePosition = ActiveSide.CENTER;

	// Hardware Objects
	private Joystick pilot;
	private Joystick copilot;

	private JoystickButton cliftUp;
	private JoystickButton cliftDn;
	private JoystickButton pliftUp;
	private JoystickButton pliftDn;
	private JoystickButton liftLowGear;
	private JoystickButton driveShiftUp;
	private JoystickButton driveShiftDn;
	private JoystickButton goGround;
	private JoystickButton goLowScale;
	private JoystickButton goSwitch;
	private JoystickButton goHighScale;
	private JoystickButton goClimbRaise;
	private JoystickButton goClimbLower;
	private JoystickButton openArms;
	private JoystickButton tiltUp;
	private JoystickButton tiltDn;
	private JoystickButton slowOut;
	private JoystickButton autoCubePickup;

	private VictorSP lDriveM;
	private VictorSP rDriveM;
	private VictorSP lCollectM;
	private VictorSP rCollectM;
	private VictorSP liftM;

	private Encoder lEncoder;
	private Encoder rEncoder;

	private DoubleSolenoid driveShiftS;
	private DoubleSolenoid liftShiftS;
	private DoubleSolenoid tiltS;
	private DoubleSolenoid mouthS;

	private DigitalInput topLimit;
	private DigitalInput bottomLimit;
	private AnalogInput liftPot;

	@Override
	public void robotInit() {

		try {
			gyro = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		// Setup Choosers
		autoDelay.addDefault(" 0", 0);
		autoDelay.addObject(" 1", 1);
		autoDelay.addObject(" 2", 2);
		autoDelay.addObject(" 3", 3);
		autoDelay.addObject(" 4", 4);
		autoDelay.addObject(" 5", 5);
		autoDelay.addObject(" 6", 6);
		autoDelay.addObject(" 7", 7);
		autoDelay.addObject(" 8", 8);
		autoDelay.addObject(" 9", 9);
		autoDelay.addObject("10", 10);
		autoDelay.addObject("11", 11);
		autoDelay.addObject("12", 12);
		SmartDashboard.putData("Auto Delay", autoDelay);

		startPosition.addObject("Left", ActiveSide.LEFT);
		startPosition.addDefault("Center", ActiveSide.CENTER);
		startPosition.addObject("Right", ActiveSide.RIGHT);
		SmartDashboard.putData("Start Position", startPosition);

		autoMode.addObject("Do Nothing", AutoMode.DO_NOTHING);
		autoMode.addDefault("Cross Line", AutoMode.CROSS_LINE);
		autoMode.addObject("Simple Switch", AutoMode.SIMPLE_SWITCH);
		autoMode.addObject("Simple Switch + 1", AutoMode.SIMPLE_SWITCH_PLUS);
		autoMode.addObject("Center Play", AutoMode.CENTER_PLAY);
		autoMode.addObject("Center Play + 1", AutoMode.CENTER_PLAY_PLUS);
		autoMode.addObject("Switch + Scale", AutoMode.SWITCH_SCALE);
		autoMode.addObject("Scale + Switch", AutoMode.SCALE_SWITCH);
		autoMode.addObject("Crossover Switch", AutoMode.CROSSOVER_SWITCH);
		autoMode.addObject("Crossover Scale", AutoMode.CROSSOVER_SCALE);
		autoMode.addObject("Long Run", AutoMode.LONG_RUN);
		SmartDashboard.putData("Auto Mode", autoMode);

		crossOver.addDefault("No", YesNo.NO);
		crossOver.addObject("Yes", YesNo.YES);
		SmartDashboard.putData("Cross Over", crossOver);

		table = NetworkTableInstance.getDefault().getTable("limelight");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
		tv = table.getEntry("tv");
		led = table.getEntry("ledMode");
		led.setValue(1);
		snapshot = table.getEntry("snapshot");

		// Setup Variables
		timer = new Timer();
		timer.start();
		autoTimer = new Timer();
		autoTimer.start();

		filteredDrive = 0;
		filteredTurn = 0;
		getExtraCube = false;

		// Setup Hardware
		lDriveM = new VictorSP(0);
		lDriveM.setInverted(true);
		rDriveM = new VictorSP(2);
		rDriveM.setInverted(false);

		lCollectM = new VictorSP(4);
		lCollectM.setInverted(true); // Positive collects
		rCollectM = new VictorSP(5);
		rCollectM.setInverted(false); // Positive collects
		liftM = new VictorSP(7);

		lEncoder = new Encoder(3, 4, false);
		lEncoder.setDistancePerPulse(0.0535);
		lEncoder.setReverseDirection(true);
		lEncoder.reset();
		rEncoder = new Encoder(5, 6, false);
		rEncoder.setDistancePerPulse(0.0535);
		rEncoder.reset();

		pilot = new Joystick(0);
		copilot = new Joystick(1);

		goClimbRaise = new JoystickButton(pilot, 1);
		pliftDn = new JoystickButton(pilot, 2);
		pliftUp = new JoystickButton(pilot, 4);
		driveShiftDn = new JoystickButton(pilot, 5);
		driveShiftUp = new JoystickButton(pilot, 6);
		autoCubePickup = new JoystickButton(pilot, 7);
		liftLowGear = new JoystickButton(pilot, 9);
		goClimbLower = new JoystickButton(pilot, 10);

		goGround = new JoystickButton(copilot, 1);
		goLowScale = new JoystickButton(copilot, 2);
		cliftDn = new JoystickButton(copilot, 3);
		slowOut = new JoystickButton(copilot, 4);
		goSwitch = new JoystickButton(copilot, 5);
		goHighScale = new JoystickButton(copilot, 6);
		cliftUp = new JoystickButton(copilot, 7);
		openArms = new JoystickButton(copilot, 8);
		tiltDn = new JoystickButton(copilot, 9);
		tiltUp = new JoystickButton(copilot, 10);

		mouthS = new DoubleSolenoid(2, 5);
		tiltS = new DoubleSolenoid(4, 3);
		driveShiftS = new DoubleSolenoid(6, 1);
		liftShiftS = new DoubleSolenoid(7, 0);

		topLimit = new DigitalInput(2);
		bottomLimit = new DigitalInput(1);
		liftPot = new AnalogInput(3);

		gyro.reset();
		initLIDAR(8);
	}

	@Override
	public void disabledInit() {
		resetEncoders();
		gyro.reset();
		headingTarget = 0;
	}

	@Override
	public void disabledPeriodic() {
		driveShiftS.set(DoubleSolenoid.Value.kForward); // Low Gear
		liftShiftS.set(DoubleSolenoid.Value.kReverse); // High Gear
		curLiftGain = LIFT_GAIN_HIGH;
		liftSetPoint = liftHeight;
		liftInPosition = false;
		goingToFloor = false;
		led.setValue(1);
	}

	@Override
	public void robotPeriodic() {

		// run the LiftControl algorithm here
		// read the string pot and determine lift height
		liftHeight = (liftPot.getAverageVoltage() * LIFT_SLOPE) - LIFT_OFFSET; // volt to inch conversion
		runLiftControl();

		// Cube Tracking Information
		target_valid = (tv.getDouble(0) != 0.0);
		target_x = tx.getDouble(0);
		target_y = ty.getDouble(0);
		
		cubeInMouth = (target_valid && (target_y < DEPTH_OF_MOUTH));

		// Update robot heading from IMU
		if (gyro != null) {
			robotHeading = gyro.getAngle();
		}

		// Determine heading error
		getHeadingError();

		// Finish go to floor action
		if (goingToFloor && liftInPosition) {
			goingToFloor = false;
			setLiftHeight(0);
		}
		
		inchesToWall = getRangeLIDAR();
		
		//Take snapshot from camera if tracking in auto or driver assist capture
		if(takeSnapshots && (trackingCube || autoCubePickup.get())) {
			snapshot.setValue(1);
		}

		// Send Variables to Dash board.
		SmartDashboard.putBoolean("TOP limit OK", topLimit.get());
		SmartDashboard.putBoolean("BOT Limit OK", bottomLimit.get());
		SmartDashboard.putBoolean("Robot Moving", robotIsMoving);
		SmartDashboard.putBoolean("Lift In Position", liftInPosition);
		SmartDashboard.putNumber("Lift Voltage", liftPot.getAverageVoltage());
		SmartDashboard.putNumber("Lift Height", liftHeight);
		SmartDashboard.putNumber("Dist to Wall", inchesToWall);
		SmartDashboard.putString("GAME", gameData);
		SmartDashboard.putString("Version", VERSION);
		
		SmartDashboard.putNumber("Lift Setpoint", liftSetPoint);
		SmartDashboard.putString("Side", selStartPosition.name());
		SmartDashboard.putString("Auto", selAutoMode.name());
		SmartDashboard.putNumber("Delay", selAutoDelay);
		
		SmartDashboard.putNumber("State", curAutoState);
		SmartDashboard.putNumber("Heading", robotHeading);
		SmartDashboard.putNumber("POV", POV);
		SmartDashboard.putBoolean("HOLDING CUBE", holdingCube);
		SmartDashboard.putBoolean("GRABBING CUBE", grabbingCube);
		SmartDashboard.putBoolean("VALID CUBE", target_valid);

	}

	@Override
	public void teleopInit() {
		driveShiftS.set(DoubleSolenoid.Value.kForward); // Low Gear
		liftShiftS.set(DoubleSolenoid.Value.kReverse); // High Gear
		led.setValue(1);
		curLiftGain = LIFT_GAIN_HIGH;
		liftSetPoint = liftHeight;
		holdingCube = false;
		resetEncoders();
		gyro.reset();
		headingTarget = 0;
	}

	@Override
	public void teleopPeriodic() {
		double forward;
		double turn;

		// Driving is either manual control (joystick) or Automatic (Cube Tracking)
		if (!autoCubePickup.get() || !target_valid) {
			trackingCube = false;
			grabbingCube = false;
			holdingCube = false;

			// Control the drive wheels based on the pilot left and right joy-stick
			// Squared inputs to give more sensitivity at low speed
			forward = -pilot.getRawAxis(1);
			turn = pilot.getRawAxis(2);
			forward = (forward * forward) * Math.signum(forward);
			turn = (turn * turn) * Math.signum(turn) * MAX_TURN_RATE;
			driveSafe(forward, turn);
		} else {
			trackingCube = true;
			if (target_valid) {
				// run a track and capture operation
				turn = 0;
				forward = 0;

				if (holdingCube) {
					// We have cube, so back up
					forward = -0.4;
					setCollector(0.2);
					setLiftHeight(LIFT_POS_DRIVE);
					
					// Check to see if it dropped out
					if (target_y > (DEPTH_OF_MOUTH + 5)) {
						grabbingCube = false;
						holdingCube = false;
						openMouth();
						setLiftHeight(0);
					}
				} else if (grabbingCube) {
					// We are in the process of closing the mouth.
					if (timer.get() > 0.5) {
						grabbingCube = false;
						holdingCube = true;
					}
				} else {
					// We don't have control yet. Turn to block
					setLiftHeight(0);

					// Do more moderate turns if we need to approach closer
					if (target_y > DISTANT_CUBE) {
						turn = clip(getHeadingError() * HEADING_DRIVE_GAIN, -1, 1) * TRACKNG_TURN_POWER;
					} else {
						turn = clip(getHeadingError() * HEADING_TURN_GAIN, -1, 1) * TRACKNG_TURN_POWER;
					}

					forward = clip(target_y * DISTANCE_GAIN, 0, 1) * TRACKING_DRIVE_POWER;

					// Grab cube if we are in range
					if (target_y < DEPTH_OF_MOUTH) {
						closeMouth();
						setCollector(1);
						grabbingCube = true;
						timer.reset();
					} else {
						openMouth();
						setCollector(0.3);
						grabbingCube = false;
					}
				}

				driveSafe(forward, turn);
			} else {
				// No target found so scan in the requested direction
				driveSafe(0.0, 0.3);
			}
		}

		// Run LIFT controls

		// Manual buttons jog the set-point.
		// Lift travels up and down if Limit Switches are true.
		if ((pliftUp.get() || cliftUp.get()) && !topLimit.get()) {
			setLiftHeight(liftSetPoint + LIFT_JOG_INCHES);
		}

		if ((pliftDn.get() || cliftDn.get()) && !bottomLimit.get()) {
			setLiftHeight(liftSetPoint - LIFT_JOG_INCHES);
		}

		// Auto buttons change set-points
		if (goHighScale.get())
			setLiftHeight(80);
		else if (goLowScale.get())
			setLiftHeight(60);
		else if (goSwitch.get())
			setLiftHeight(24);
		else if (goGround.get())
			setLiftHeight(0); // Indicate going to floor
		else if (goClimbRaise.get())
			setLiftHeight(LIFT_POS_CLIMB_RAISE);

		// Run collector motors
		// Note: This manual control is ignored when tracking a cube
		if (!trackingCube) {
			POV = copilot.getPOV();
			if (POV != -1) {
				mouthS.set(DoubleSolenoid.Value.kReverse); // Close arms

				if ((POV == 0) || (POV == 45) || (POV == 315)) {
					if (slowOut.get()) {
						lCollectM.set(SLOW_PUSH);
						rCollectM.set(SLOW_PUSH);
					} else {
						lCollectM.set(EJECT_SPEED);
						rCollectM.set(EJECT_SPEED);
					}
					holdingCube = false;
					grabbingCube = false;
				} else if ((POV == 180) || (POV == 225) || (POV == 135)) {
					lCollectM.set(COLLECT_SPEED);
					rCollectM.set(COLLECT_SPEED);
				} else if (POV == 90) {
					lCollectM.set(ROTATION_SPEED);
					rCollectM.set(COLLECT_SPEED);
				} else if (POV == 270) {
					lCollectM.set(COLLECT_SPEED);
					rCollectM.set(ROTATION_SPEED);
				}

			} else {
				lCollectM.set(0.0);
				rCollectM.set(0.0);
			}
		}

		// Run Pneumatics
		if (driveShiftDn.get())
			driveShiftS.set(DoubleSolenoid.Value.kForward);
		else if (driveShiftUp.get())
			driveShiftS.set(DoubleSolenoid.Value.kReverse);

		if (liftLowGear.get()) {
			liftShiftS.set(DoubleSolenoid.Value.kForward);
			curLiftGain = LIFT_GAIN_LOW;

			if (goClimbLower.get())
				setLiftHeight(LIFT_POS_CLIMB_LOWER);

		} else {
			liftShiftS.set(DoubleSolenoid.Value.kReverse);
			curLiftGain = LIFT_GAIN_HIGH;
		}

		// Collector Tilt Mechanism
		if (tiltUp.get() || (pilot.getPOV() == 0))
			tiltS.set(DoubleSolenoid.Value.kForward);
		else if (tiltDn.get() || (pilot.getPOV() == 180))
			tiltS.set(DoubleSolenoid.Value.kReverse);
		else
			tiltS.set(DoubleSolenoid.Value.kOff);

		if (openArms.get())
			mouthS.set(DoubleSolenoid.Value.kForward);
	}

	@Override
	public void autonomousInit() {
		driveShiftS.set(DoubleSolenoid.Value.kForward); // Low Gear
		liftShiftS.set(DoubleSolenoid.Value.kReverse); // High Gear
		curLiftGain = LIFT_GAIN_HIGH;
		liftSetPoint = liftHeight;
		tiltS.set(DoubleSolenoid.Value.kForward); // Tilt Up
		mouthS.set(DoubleSolenoid.Value.kReverse); // Mouth Closed
		curAutoState = 0;

		// Read game data and set switch/scale positions
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData.charAt(0) == 'L')
				switchPosition = ActiveSide.LEFT;
			else
				switchPosition = ActiveSide.RIGHT;

			if (gameData.charAt(1) == 'L')
				scalePosition = ActiveSide.LEFT;
			else
				scalePosition = ActiveSide.RIGHT;
		}

		// Read Auto settings
		selStartPosition = startPosition.getSelected();
		selAutoDelay = autoDelay.getSelected();
		selCrossOver = crossOver.getSelected();

		// Read and process Auto Modes
		selAutoMode = autoMode.getSelected();
		if ((selAutoMode == AutoMode.SIMPLE_SWITCH_PLUS) || (selAutoMode == AutoMode.CENTER_PLAY_PLUS)) {
			getExtraCube = true;
		} else {
			getExtraCube = false;
		}

		// if Switch/Scale is selected, but we don't have Switch, change path
		if ((selAutoMode == AutoMode.SWITCH_SCALE) 	&& !(selStartPosition.equals(switchPosition))) {

			// If crossover mode is selected, do the switch on the other side
			if (selCrossOver == YesNo.YES) {
				selAutoMode = AutoMode.CROSSOVER_SWITCH;
			} else {
				// If we have the scale, do a ScaleSwitch
				if (selStartPosition.equals(scalePosition)) {
					selAutoMode = AutoMode.SCALE_SWITCH;
				} else {
					// Just do the minimum
					selAutoMode = AutoMode.CROSS_LINE;
				}
			}
		}
		
		// If Scale Switch is selected, but we have neither, check for Crossover
		if ((selAutoMode == AutoMode.SCALE_SWITCH) && 
				!(selStartPosition.equals(switchPosition) || selStartPosition.equals(scalePosition))) {
			if (selCrossOver == YesNo.YES) {
					selAutoMode = AutoMode.CROSSOVER_SCALE;
			} 
		}

		// if Crossover Switch is selected, but switch is on our side, just do Switch Scale
		if ((selAutoMode == AutoMode.CROSSOVER_SWITCH) 	&& selStartPosition.equals(switchPosition)) {
			selAutoMode = AutoMode.SWITCH_SCALE	;
		}
		
		// if Crossover Scale is selected, but scale is on our side, just do Scale Switch
		if ((selAutoMode == AutoMode.CROSSOVER_SCALE) 	&& selStartPosition.equals(scalePosition)) {
			selAutoMode = AutoMode.SCALE_SWITCH	;
		}
		
		
		// if Long Run is selected, but scale isn't on our side, check for switch. If switch isn't on out side, just cross line
		if ((selAutoMode == AutoMode.LONG_RUN) && !(selStartPosition.equals(scalePosition))) {
			if (selStartPosition.equals(switchPosition)) 
				selAutoMode = AutoMode.SIMPLE_SWITCH;
			else 
				selAutoMode = AutoMode.CROSS_LINE;
		}
		
		
		
		// Reset all drive parameters
		resetEncoders();
		gyro.reset();
		autoTimer.reset();

		// Prep the collector
		tiltS.set(DoubleSolenoid.Value.kReverse); // Tilt Down
		closeMouth();
		setLiftHeight(LIFT_POS_DRIVE); // Don't drag on the floor

	}

	@Override
	public void autonomousPeriodic() {
		// Run Auto once auto delay has expired
		if (autoTimer.get() > selAutoDelay) {
			switch (selAutoMode) {
				case DO_NOTHING: // do Nothing
				default:
					break;
	
				case CROSS_LINE: // cross line
					autoCrossLine();
					break;
	
				case SIMPLE_SWITCH: // simple switch
				case SIMPLE_SWITCH_PLUS: // simple switch
					autoSimpleSwitch();
					break;
	
				case CENTER_PLAY: // starts in center and scores based on game data
				case CENTER_PLAY_PLUS: // starts in center and scores based on game data
					autoCenterPlay();
					break;
	
				case SCALE_SWITCH: // Scores Scale then Switch
					autoScaleSwitch();
					break;
	
				case SWITCH_SCALE: // Scored Switch then scale
					autoSwitchScale();
					break;
					
				case CROSSOVER_SWITCH:
					autoCrossoverSwitch();
					break;
					
				case LONG_RUN:
					autoLongRun();
					
				case CROSSOVER_SCALE:
					autoCrossoverScale();
					break;
			}
		}
	}

	// ----------------------------------------------------------------------------------------
	// Autonomous Mode Methods
	// ----------------------------------------------------------------------------------------
	void autoCrossLine() {
		switch (curAutoState) {
		case 0:
			// Start the robot moving then break;
			driveRobot(105, 0.6, 0, 4.0, BRAKE);
			curAutoState = 100;
			break;

		case 100:
			// Drive until we have gone required distance
			if (moveIsComplete()) {
				stopRobot();
				setLiftHeight(0);
				curAutoState = 101;
			}
			break;

		case 101:
		default:
			stopRobot();
			break;
		}
	}

	// ===============================================================================================
	void autoSimpleSwitch() {
		switch (curAutoState) {
		case 0:
			// Start the robot moving
			setLiftHeight(LIFT_POS_AUTO_SWITCH);
			if (selStartPosition == ActiveSide.RIGHT)
				driveRobot(80, 0.7, 0, 2.5);
			else
				driveRobot(80, 0.7, 10, 2.5);
			curAutoState = 1;
			break;

		case 1:
			if (moveIsComplete()) {
				// Drive square into Switch
				driveRobot(25, 0.7, 0, 2.0, BRAKE); // wait for contact
				curAutoState = 2;
			}
			break;

		case 2:
			if (moveIsComplete()) {
				// Stop Robot and score if we have a color match
				stopRobot(); // Stop robot
				curAutoState = 3; // regular exit

				// check for matching start and switch sides
				if (selStartPosition.equals(switchPosition)) {
					setCollector(-0.50);
					if (getExtraCube) {
						getExtraCube = false;
						curAutoState = 10; // Get extra cube sequence
					}
				}
			}
			break;

		case 3:
			if (timerExpired(0.5)) {
				// Stop Collector and back away
				setCollector(0);
				driveRobot(24, -0.6, 0, 2.0, BRAKE);
				curAutoState = 100; // ===== EXIT =====
			}
			break;

		// Code to get extra cube
		case 10:
			if (timerExpired(0.25)) {
				// Stop collector and prepare for next Block
				setCollector(0);
				setLiftHeight(20);
				driveRobot(48, -0.7, 0, 2.0, BRAKE);
				curAutoState = 11;
			}
			break;

		case 11:
			if (moveIsComplete()) {
				// Set lift for pickup and turn to blocks
				setLiftHeight(LIFT_POS_STACK);

				if (switchPosition == ActiveSide.RIGHT)
					turnRobot(-45, 0.4, 1.5);
				else
					turnRobot(45, 0.4, 1.5);
				curAutoState = 12;
			}
			break;

		case 12:
			if (moveIsComplete()) {
				// Start Collector and drive into stack
				setCollector(0.75);
				driveRobot(50, 0.6, headingTarget, 3);
				curAutoState = 13;
			}
			break;

		case 13:
			if (moveIsComplete()) {
				// more power to collector
				setCollector(1.0);
				driveRobot(14, 0.3, headingTarget, 3, BRAKE);
				curAutoState = 14;
			}
			break;

		case 14:
			if (moveIsComplete()) {
				// Slow down collector and back out
				setCollector(0.5);
				driveRobot(24, -0.4, headingTarget, 4);
				curAutoState = 15;
			}
			break;

		case 15:
			if (moveIsComplete()) {
				setLiftHeight(LIFT_POS_DRIVE);
				// Slow down collector and back out
				setCollector(0.25);
				driveRobot(26, -0.7, headingTarget, 4, BRAKE);
				curAutoState = 16;
			}
			break;

		case 16:
			if (moveIsComplete()) {
				// Turn back to switch
				turnRobot(0, 0.4, 2);
				curAutoState = 17;
			}
			break;

		case 17:
			if (moveIsComplete()) {
				// Raise lift and drive to switch
				setLiftHeight(LIFT_POS_AUTO_SWITCH);
				driveRobot(30, 0.4, 0, 2, BRAKE);
				curAutoState = 1; // revert back to normal scoring
			}
			break;

		case 100:
			// stop and reset height
			if (moveIsComplete()) {
				stopRobot();
				setLiftHeight(0);
				curAutoState = 101;
			}
			break;

		case 101:
		default:
			stopRobot();
			break;
		}
	}

	// ===============================================================================================
	void autoCenterPlay() {
		switch (curAutoState) {
		case 0:
			// Start the robot moving
			driveRobot(12, 0.4, 0, 2.0);
			curAutoState = 1;
			break;

		case 1:
			if (moveIsComplete()) {
				// Turn to correct switch half
				if (switchPosition == ActiveSide.RIGHT)
					turnRobot(30, 0.35, 2);
				else
					turnRobot(-35, 0.35, 2);
				curAutoState = 2;
			}
			break;

		case 2:
			if (moveIsComplete()) {
				// Drive to outside
				setLiftHeight(LIFT_POS_SWITCH);
				driveRobot(86, 0.6, headingTarget, 5);
				curAutoState = 3;
			}
			break;

		case 3:
			if (moveIsComplete()) {
				// Turn back towards switch
				turnRobot(0, 0.3, 2.0);
				curAutoState = 4;
			}
			break;

		case 4:
			if (moveIsComplete()) {
				// Lift to final height and drive to switch
				setLiftHeight(LIFT_POS_AUTO_SWITCH);
				driveRobot(24, 0.5, 0, 3, BRAKE); // Wait for contact
				curAutoState = 5;
			}
			break;

		case 5:
			if (moveIsComplete()) {
				// Shoot block
				setCollector(-0.50);
				stopRobot();

				if (getExtraCube) {
					curAutoState = 10; // Get extra cube sequence
					getExtraCube = false;
				} else {
					curAutoState = 6;
				}
			}
			break;

		case 6:
			if (timerExpired(0.25)) {
				// Back Away after short shoot delay
				stopRobot();
				driveRobot(24, -0.3, 0, 2);
				setCollector(0);
				curAutoState = 100; // ===== EXIT =====
			}
			break;

		// ---------- Code to get extra cube
		// Code to get extra cube
		case 10:
			if (timerExpired(0.25)) {
				// Stop collector and prepare for next Block
				setCollector(0);
				setLiftHeight(20);
				driveRobot(48, -0.7, 0, 2.0, BRAKE);
				curAutoState = 11;
			}
			break;

		case 11:
			if (moveIsComplete()) {
				// Set lift for pickup and turn to blocks
				setLiftHeight(LIFT_POS_STACK);

				if (switchPosition == ActiveSide.RIGHT)
					turnRobot(-45, 0.4, 1.5);
				else
					turnRobot(45, 0.4, 1.5);
				curAutoState = 12;
			}
			break;

		case 12:
			if (moveIsComplete()) {
				// Start Collector and drive into stack
				setCollector(0.5);
				driveRobot(50, 0.5, headingTarget, 3); 
				curAutoState = 13;
			}
			break;

		case 13:
			if (moveIsComplete()) {
				// Slow down
				setCollector(1.0);
				driveRobot(14, 0.3, headingTarget, 3, BRAKE);
				curAutoState = 14;
			}
			break;

		case 14:
			if (moveIsComplete()) {
				// back out
				driveRobot(24, -0.4, headingTarget, 4);
				curAutoState = 15;
			}
			break;

		case 15:
			if (moveIsComplete()) {
				setLiftHeight(LIFT_POS_DRIVE);
				// Slow down collector and keep reversing
				setCollector(0.25);
				driveRobot(26, -0.7, headingTarget, 4, BRAKE);
				curAutoState = 16;
			}
			break;

		case 16:
			if (moveIsComplete()) {
				// Turn back to switch
				setCollector(0.25);
				turnRobot(0, 0.4, 2);
				curAutoState = 17;
			}
			break;

		case 17:
			if (moveIsComplete()) {
				// Raise lift and drive to switch
				setLiftHeight(LIFT_POS_AUTO_SWITCH);
				driveRobot(30, 0.4, 0, 2);  			// There used to be a BRAKE here, but it should not be needed.
				curAutoState = 4; // revert back to normal scoring
			}
			break;
		// ---------- End of Code to get extra cube

		case 100:
			if (moveIsComplete()) {
				stopRobot();
				setLiftHeight(0);
				curAutoState = 101;
			}
			break;

		case 101:
		default:
			stopRobot();
			break;
		}
	}

	// ===============================================================================================
	void autoScaleSwitch() {
		switch (curAutoState) {
		case 0:
			// Start the robot moving
			if (selStartPosition.equals(scalePosition)) {
				driveRobot(290, 0.90, 0, 8.0, BRAKE);
				curAutoState = 2;
			} else if (selStartPosition.equals(switchPosition)) {
				driveRobot(144, 0.8, 0, 4.0, BRAKE);
				curAutoState = 21; // Score switch
			} else {
				driveRobot(100, 0.6, 0, 4.0, BRAKE);
				setLiftHeight(LIFT_POS_SWITCH); // Raise to score switch later
				curAutoState = 101; // ===== EXIT =====
			}
			break;

		// This code scores in the scale
		case 2:
			if (moveIsComplete()) {
				// Turn to Scale
				if (selStartPosition == ActiveSide.RIGHT)
					turnRobot(-90, 0.4, 2);
				else
					turnRobot(90, 0.4, 2);
				curAutoState = 3;
			}
			break;

		case 3:
			if (moveIsComplete()) {
				// Backup to wall
				setLiftHeight(LIFT_POS_SCALE_LOW);
				driveRobot(36, -0.3, headingTarget, 2);
				curAutoState = 4;
			}
			break;

		case 4:
			if (moveIsComplete()) {
				//Drive to scale
				driveRobot(26, 0.5, headingTarget, 2.0, BRAKE);
				setLiftHeight(LIFT_POS_SCALE_HIGH);
				curAutoState = 5;
			}
			break;

		case 5:
			// getting closer while lifting
			if (moveIsComplete()) {
				stopRobot();
				curAutoState = 6;
			}
			break;

		case 6:
			if (liftIsReady(0.5)) {
				// Score cube
				setCollector(-0.85);
				timer.reset();
				curAutoState = 7;
			}
			break;

		case 7:
			if (timerExpired(0.10)) {
				// Stop shooter and drop lift
				setCollector(0);
				setLiftHeight(0);
				curAutoState = 8;
			}
			break;

		case 8:
			if (timerExpired(0.5)) {
				// Lift mostly down, turn to pickup new cube.
				if (selStartPosition == ActiveSide.RIGHT)
					turnRobot(-158, 0.4, 2);
				else
					turnRobot(158, 0.4, 2);
				curAutoState = 9;
			}
			break;

		case 9:
			if (moveIsComplete()) {
				// Drive to cube with tracking on (fast)
				openMouth();
				setLiftHeight(0);
				trackingCube = true;
				driveRobot(80, 0.7, headingTarget, 2);
				curAutoState = 10;
			}
			break;

		case 10:
			if (moveIsComplete()) {
				// Drive to cube with tracking on (slow)
				trackingCube = true;
				setCollector(1.0);
				driveRobot(32, 0.4, headingTarget, 2, BRAKE);
				curAutoState = 11;
			}
			break;

		case 11:
			if (moveIsComplete() || cubeInMouth) {
				// Close collector onto cube and wait to grab
				// Lock onto current heading
				trackingCube = false;
				headingTarget = robotHeading;
				stopRobot();
				closeMouth();
				setCollector(1.0);
				curAutoState = 12;
			}
			break;

		case 12:
			if (timerExpired(0.3)) {
				// Back away from switch
				driveRobot(2, -0.3, headingTarget, 1);
				curAutoState = 13;
			}
			break;
			
		case 13:
			if (moveIsComplete()) {
				// Move lift after backing away
				setLiftHeight(LIFT_POS_AUTO_SWITCH);
				curAutoState = 14;
			}
			break;

		case 14:
			if (moveIsComplete()) {

				if (selStartPosition.equals(switchPosition)) {
					// Drive into switch for score position
					stopRobot();
					setCollector(0.75);
					curAutoState = 15; // was 14 by mistake 
				}
				else {
					// Just wait here for Auto to end
					stopRobot();
					curAutoState = 101;
					setCollector(0);
				}
			}
			break;
			
		case 15:
			if (liftIsReady(1.0)) {
				// Drive into switch for score position
				driveRobot(8, 0.6, headingTarget, 0.5, BRAKE);
				setCollector(0.0);
				curAutoState = 16;
			}
			break;

		case 16:
			if (moveIsComplete()) {
				// Shoot block and roll back
				setCollector(-1.0);
				driveRobot(12, -0.5, headingTarget, 1, BRAKE);
				curAutoState = 100;
			}
			break;

		// --------------- This code scores in the switch
		case 21:
			if (moveIsComplete()) {
				// Turn to switch
				if (selStartPosition == ActiveSide.RIGHT)
					turnRobot(-90, 0.4, 2);
				else
					turnRobot(90, 0.4, 2);
				curAutoState = 22;
			}
			break;

		case 22:
			if (moveIsComplete()) {
				// Lift Cube and wait
				stopRobot();
				setLiftHeight(LIFT_POS_AUTO_SWITCH); // Raise to score switch
				curAutoState = 23;
			}
			break;

		case 23:
			if (liftIsReady(1)) {
				// drive up to switch
				driveRobot(30, 0.5, headingTarget, 3, BRAKE);
				curAutoState = 24;
			}
			break;

		case 24:
			if (moveIsComplete()) {
				stopRobot(); // Stop robot
				setCollector(-0.50);
				curAutoState = 25;
			}
			break;

		case 25:
			if (timer.get() > 0.5) {
				// Stop shooter and roll back
				setCollector(0);
				driveRobot(24, -0.25, headingTarget, 2.0);
				curAutoState = 100; // ===== EXIT =====
			}
			break;

		// ----------------------------------------------------------------------------
		// This is the ending code for all sequences
		case 100:
			if (moveIsComplete()) {
				stopRobot();
				setLiftHeight(0);
				curAutoState = 101;
			}
			break;

		case 101:
		default:
			stopRobot();
			break;
		}
	}

	// ===============================================================================================
	void autoSwitchScale() {
		switch (curAutoState) {
		case 0:
			// Start the robot moving
			driveRobot(200, 0.9, 0, 5.0);
			curAutoState = 3;
			break;

		case 3:
			if (moveIsComplete()) {
				// Turn towards center of field
				if (selStartPosition == ActiveSide.RIGHT)
					driveRobot(70, 0.50, -90, 3, BRAKE);
				else
					driveRobot(70, 0.50, 90, 3, BRAKE);
				curAutoState = 5;
			}
			break;

		case 5:
			if (moveIsComplete()) {
				// Turn towards switch
				if (selStartPosition == ActiveSide.RIGHT)
					turnRobot(-160, 0.40, 2);
				else
					turnRobot(160, 0.40, 2);
				setLiftHeight(LIFT_POS_SWITCH);
				curAutoState = 6;
			}
			break;

		case 6:
			if (moveIsComplete()) {
				// Raise lift to score cube
				setLiftHeight(LIFT_POS_AUTO_SWITCH);
				curAutoState = 7;
			}
			break;

		case 7:
			if (liftIsReady(0.5)) {
				driveRobot(30, 0.5, headingTarget, 2, BRAKE);
				curAutoState = 8;
			}
			break;

		case 8:
			if (moveIsComplete()) {
				// Score first Cube then back out
				setCollector(-0.75); // shoot cube
				driveRobot(22, -0.5, headingTarget, 2, BRAKE);
				curAutoState = 10;
			}
			break;

		case 10:
			if (moveIsComplete()) {
				// Stop collector and lower lift
				setCollector(1);
				setLiftHeight(1);
				stopRobot();
				openMouth();
				setCollector(1.0); // Grab cube
				curAutoState = 11;
			}
			break;

		case 11:
			if (liftIsReady(0.5)) {
				// Drive forward while searching for cube
				setLiftHeight(0);
				trackingCube = true;
				driveRobot(22, 0.5, headingTarget, 2, BRAKE);
				setCollector(1.0); // Grab cube
				curAutoState = 12;
			}
			break;

		case 12:
			if (moveIsComplete() || cubeInMouth) {
				// Close mouth to grab cube, back up
				trackingCube = false;
				closeMouth();
				if (selStartPosition == ActiveSide.RIGHT)
					headingTarget = -180;
				else
					headingTarget = 180;

				driveRobot(6, -0.2, headingTarget, 1); // Set short timeout
				curAutoState = 14;
			}
			break;

		case 14:
			if (moveIsComplete()) {
				setCollector(0.2); // Hold cube

				// Score scale if possible
				if (selStartPosition.equals(scalePosition)) {
					// Turn to Scale
					if (selStartPosition == ActiveSide.RIGHT)
						turnRobot(-10, 0.4, 2);
					else
						turnRobot(10, 0.4, 2);
					setLiftHeight(LIFT_POS_SWITCH);
					curAutoState = 20;
				} else {
					// Score back in switch
					stopRobot();
					setLiftHeight(LIFT_POS_AUTO_SWITCH);
					curAutoState = 30;
				}
			}
			break;

		// --------------------------------------------------------------------
		// Score second cube in Scale
		case 20:
			if (moveIsComplete()) {
				// Drive to scale and lift up
				driveRobot(36, 0.5, headingTarget, 4, BRAKE); // Set short timeout
				setLiftHeight(LIFT_POS_SCALE_HIGH);
				curAutoState = 21;
			}
			break;

		case 21:
			if (moveIsComplete()) {
				// Stop and Lift up remaining height
				stopRobot();
				curAutoState = 22;
			}
			break;

		case 22:
			if (liftIsReady(1.0)) {
				// Move forward
				setCollector(-0.75);
				driveRobot(6, 0.25, headingTarget, 1); // Set short timeout
				curAutoState = 23;
			}
			break;

		case 23:
			if (moveIsComplete()) {
				// Eject power cube into scale, then back up
				driveRobot(6, -0.25, headingTarget, 1); // Set short timeout
				curAutoState = 100; // ===== EXIT =====
			}
			break;

		// --------------------------------------------------------------------
		// Score Second Power Cube in Switch
		case 30:
			if (liftIsReady(1.0)) {
				// Drive back to switch
				driveRobot(36, 0.6, headingTarget, 2, BRAKE);
				setLiftHeight(LIFT_POS_AUTO_SWITCH);
				curAutoState = 31;
			}
			break;

		case 31:
			if (moveIsComplete()) {
				// Score cube in Switch then back away
				setCollector(-0.55);
				driveRobot(14, -0.4, headingTarget, 1.0); // Set short timeout
				curAutoState = 100;
			}
			break;

		// --------------------------------------------------------------------
		case 100:
			if (moveIsComplete()) {
				// Ending
				lCollectM.set(0);
				rCollectM.set(0);
				stopRobot();
				setLiftHeight(0);
				curAutoState = 101;
			}
			break;

		case 101:
		default:
			stopRobot();
			break;
		}
	}

	// ===============================================================================================
	void autoCrossoverSwitch() {
		switch (curAutoState) {
		case 0:
			// Start the robot moving
			driveRobot(226, 0.9, 0, 4.0, BRAKE);
			curAutoState = 1;
			break;

		case 1:
			if (moveIsComplete()) {
				// Turn to cross field
				if (selStartPosition == ActiveSide.RIGHT)
					turnRobot(-90, 0.4, 2);
				else
					turnRobot(90, 0.4, 2);
				curAutoState = 2;
			}
			break;

		case 2:
			if (moveIsComplete()) {
				// drive across back of switch
				driveRobot(178, 0.8, headingTarget, 5, BRAKE);
				setLiftHeight(LIFT_POS_SWITCH);
				curAutoState = 3;
			}
			break;

		case 3:
			if (moveIsComplete()) {
				// Turn towards Switch
				if (selStartPosition == ActiveSide.RIGHT)
					turnRobot(-180, 0.4, 2);
				else
					turnRobot(180, 0.4, 2);
				curAutoState = 4;
			}
			break;

		case 4:
			if (moveIsComplete()) {
				// Drive into switch
				setLiftHeight(LIFT_POS_AUTO_SWITCH);
				driveRobot(24, 0.5, headingTarget, 2.0, BRAKE);
				curAutoState = 5;
			}
			break;

		case 5:
			if (moveIsComplete()) {
				// Turn towards Switch (again since we may be skewed)
				if (selStartPosition == ActiveSide.RIGHT)
					turnRobot(-180, 0.4, 1);
				else
					turnRobot(180, 0.4, 1);
				curAutoState = 6;
			}
			break;

		case 6:
			if (moveIsComplete()) {
				// shoot and Back out
				setCollector(-0.85);
				driveRobot(20, -0.4, headingTarget, 1.5); // Set short timeout
				curAutoState = 10;
			}
			break;

		//  Ready for second cube
		case 10:
			if (moveIsComplete()) {
				// Drop collector and try for another cube
				stopRobot();
				setLiftHeight(0);
				setCollector(0.1);
				openMouth();
				trackingCube = true;
				curAutoState = 11;
			}
			break;

		case 11:
			if (liftIsReady(0.5)) {
				setCollector(1.0);
				driveRobot(20, 0.4, headingTarget, 2.0); 
				curAutoState = 12;
			}
			break;

		case 12:
			if (moveIsComplete() || cubeInMouth ) {
				// Grab the cube that is in out grasp
				stopRobot();
				closeMouth();
				trackingCube = false;
				curAutoState = 13;
			}
			break;

		case 13:
			if (timerExpired(0.25)) {
				// We've got it so back out to clear switch
				setCollector(1.0);
				setLiftHeight(LIFT_POS_AUTO_SWITCH);
//				driveRobot(2, -0.4, headingTarget, 1.0); // Set short timeout
				curAutoState = 15;
			}
			break;

		case 14:
			if (moveIsComplete()) {
				// Switch is cleared (don't strip zip ties)
				setCollector(0.5);
				stopRobot();
				curAutoState = 15;
			}
			break;

		case 15:
			if (liftIsReady(1.0)) {
				// We are high enough to drive forward to get in range
				setCollector(0.1);
				driveRobot(8, 0.4, headingTarget, 2.0); 
				curAutoState = 16;
			}
			break;

		case 16:
			if (moveIsComplete()) {
				// Score the cube and back out
				setCollector(-1.0);
				driveRobot(12, -0.3, headingTarget, 2.0); 
				curAutoState = 100;
			}
			break;

		case 100:
			// Ending
			if (moveIsComplete()) {
				setCollector(0);
				setLiftHeight(0);
				stopRobot();
				curAutoState = 101;
			}
			break;

		case 101:
		default:
			setCollector(0.0);
			stopRobot();
			break;

		}
	}

	// ===============================================================================================
	void autoCrossoverScale() {
		switch (curAutoState) {
		case 0:
			// Start the robot moving
			driveRobot(226, 0.9, 0, 4.0, BRAKE);
			curAutoState = 1;
			break;

		case 1:
			if (moveIsComplete()) {
				// Turn to cross field
				if (selStartPosition == ActiveSide.RIGHT)
					turnRobot(-90, 0.4, 2);
				else
					turnRobot(90, 0.4, 2);
				curAutoState = 2;
			}
			break;

		case 2:
			if (moveIsComplete()) {
				// drive across back of switch
				driveRobot(202, 0.8, headingTarget, 5, BRAKE);
				curAutoState = 3;
			}
			break;

		case 3:
			if (moveIsComplete()) {
				// Turn towards Scale
				setLiftHeight(LIFT_POS_SWITCH);
				turnRobot(0, 0.4, 2);
				curAutoState = 4;
			}
			break;

		case 4:
			if (moveIsComplete()) {
				// Drive close to scale but not close enough to hit. 
				setLiftHeight(LIFT_POS_SCALE_HIGH);
				driveRobot(30, 0.5, headingTarget, 2.0, BRAKE); // ### Go LESS if we hit ###
				curAutoState = 5;
			}
			break;
			
		case 5:
			if (moveIsComplete()) {
				// pause for lift to get to position
				stopRobot();
				curAutoState = 6;
			}
			break;
			

		case 6:
			if (liftIsReady(1.0)) {
				// drive across Null zone line to ensure legal score
				driveRobot(16, 0.5, headingTarget, 2, BRAKE);  // ### Go MORE if we don't reach ###
				curAutoState = 7;
			}
			break;

		case 7:
			if (moveIsComplete()) {
				// shoot and Back out
				setCollector(-0.5); // Not too hard to overshoot
				driveRobot(10, -0.4, headingTarget, 1.0, BRAKE); 
				curAutoState = 10;
			}
			break;

		// Go for second cube
		case 10:
			if (moveIsComplete()) {
				// Drop collector and try for another cube
				stopRobot();
				setLiftHeight(0);
				setCollector(0);
				curAutoState = 11;
			}
			break;

		case 11:
			if (liftIsReady(0.5)) {
				// Turn towards Switch 
				setLiftHeight(0);
				
				// Turn towards Switch (again since we may be skewed)
				if (selStartPosition == ActiveSide.RIGHT)
					turnRobot(170, 0.4, 2);
				else
					turnRobot(-170, 0.4, 2);
				curAutoState = 12;
			}
			break;


		case 12:
			if (moveIsComplete()) {
				// Spin up collector for another cube, and drive forward with open mouth
				// Use camera to locate cube
				trackingCube = true;
				setCollector(0.5);
				openMouth();
				driveRobot(42, 0.6, headingTarget, 2.0, BRAKE); 
				curAutoState = 13;
			}
			break;

		case 13:
			if (moveIsComplete() || cubeInMouth ) {
				// close mouth and grab cube
				closeMouth();
				setCollector(1.0);
				stopRobot();
				trackingCube = false;
				curAutoState = 14;
			}
			break;

		case 14:
			if (timerExpired(0.20)) {
				// Raise the cube up to score
				setCollector(1.0);
				setLiftHeight(LIFT_POS_AUTO_SWITCH);
//				driveRobot(2, -0.4, headingTarget, 1.0); // Set short timeout 
				
				// Attempt to score if we have the switch on this side (opposite to start)
				if (!selStartPosition.equals(switchPosition)) {
					curAutoState = 16;
				}
				else {
					curAutoState = 101;
				}
			}
			break;

		case 15:
			// Switch is cleared (don't strip zip ties)
			if (moveIsComplete()) {
				setCollector(0.5);
				stopRobot();
				curAutoState = 16;
			}
			break;

		case 16:
			if (liftIsReady(1.0)) {
				// We are high enough to drive forward to get in range
				setCollector(0.1);
				driveRobot(12, 0.3, headingTarget, 2.0); 
				curAutoState = 17;
			}
			break;

		case 17:
			if (moveIsComplete()) {
				// Score the cube and back out
				setCollector(-1.0);
				driveRobot(12, -0.3, headingTarget, 2.0); 
				curAutoState = 100;
			}
			break;

		case 100:
			// Ending
			if (moveIsComplete()) {
				setCollector(0);
				setLiftHeight(0);
				stopRobot();
				curAutoState = 101;
			}
			break;

		case 101:
		default:
			setCollector(0.0);
			stopRobot();
			break;

		}
	}
	
	void autoLongRun() {
		switch (curAutoState) {
			case 0:
				driveRobot(290, 0.90, 0, 8.0, BRAKE);
				curAutoState = 2;
				break;
			case 2:
				if (moveIsComplete()) {
					// Turn to Scale
					if (selStartPosition == ActiveSide.RIGHT)
						turnRobot(-90, 0.4, 2);
					else
						turnRobot(90, 0.4, 2);
					curAutoState = 3;
				}
				break;

			case 3:
				if (moveIsComplete()) {
					// Backup to wall
					setLiftHeight(LIFT_POS_SCALE_LOW);
					driveRobot(36, -0.3, headingTarget, 2);
					curAutoState = 4;
				}
				break;

			case 4:
				if (moveIsComplete()) {
					//Drive to scale
					driveRobot(26, 0.5, headingTarget, 2.0, BRAKE);
					setLiftHeight(LIFT_POS_SCALE_HIGH);
					curAutoState = 5;
				}
				break;

			case 5:
				// getting closer while lifting
				if (moveIsComplete()) {
					stopRobot();
					curAutoState = 6;
				}
				break;

			case 6:
				if (liftIsReady(0.5)) {
					// Score cube
					setCollector(-0.85);
					timer.reset();
					curAutoState = 7;
				}
				break;

			case 7:
				if (timerExpired(0.10)) {
					// Stop shooter and drop lift
					setCollector(0);
					setLiftHeight(0);
					curAutoState = 100;
				}
				break;
			case 100:
				if (moveIsComplete()) {
					// Ending
					lCollectM.set(0);
					rCollectM.set(0);
					stopRobot();
					setLiftHeight(0);
					curAutoState = 101;
				}
				break;

			case 101:
			default:
				stopRobot();
				break;
			}
		}
	

	// ----------------------------------------------------------------------------------------
	// Helper Methods
	// ----------------------------------------------------------------------------------------

	// Built in range clipping method
	double clip(double val, double min, double max) {
		return (Math.max(min, Math.min(max, val)));
	}

	// driveSafe ensures that the robot won't tip if the lift is raised.
	// It does this by applying acceleration limits based on lift height.
	void driveSafe(double drive, double turn) {
		// Determine time constant for drive and turn based on height
		double safeFraction = 1 - (liftHeight / MAX_HEIGHT);
		double driveTC = 0.1;
		double turnTC = 0.2;

		if (liftHeight > 12) {
			driveTC = 0.015 + (safeFraction * 0.06);
			turnTC = 0.015 + (safeFraction * 0.06);
		}

		// reduce the acceleration (rate of change of speed) based on lift height
		filteredDrive += ((drive - filteredDrive) * driveTC);
		filteredTurn += ((turn - filteredTurn) * turnTC);

		// Output derived wheel powers
		lDriveM.set(filteredDrive + filteredTurn);
		rDriveM.set(filteredDrive - filteredTurn);
	}

	// Get the average distance traveled by each wheel
	double inchesTraveled() {

		double distanceTravelled = (Math.abs(lEncoder.getDistance()) + Math.abs(rEncoder.getDistance())) / 2;

		// Wait for a short time before really checking for moving condition.
		robotIsMoving = ((timer.get() < 0.5)
				|| (Math.abs(distanceTravelled - lastDistanceTraveled) > MAX_STOPPED_DISTANCE));

		lastDistanceTraveled = distanceTravelled; // Save last movement distance for next time around the loop

		return (distanceTravelled);
	}

	// Moves the robot according to power and heading parameters no-braking (by
	// default)
	void driveRobot(double inches, double fwd, double heading, double timeoutS) {
		driveRobot(inches, fwd, heading, timeoutS, false);
	}

	// Moves the robot according to power and heading parameters (optional braking)
	void driveRobot(double inches, double fwd, double heading, double timeoutS, boolean brake) {
		resetEncoders();
		moveRobot(fwd, 0);
		distanceTarget = inches;
		headingTarget = heading;
		drivePower = fwd;
		timeout = timeoutS;
		if (brake) {
			driveMode = AutoDriveMode.DRIVING_BRAKE;
		} else {
			driveMode = AutoDriveMode.DRIVING;
		}
	}

	// Drives the robot according to turn parameter (and alliance color).
	void turnRobot(double heading, double turn, double timeoutS) {
		resetEncoders();
		moveRobot(0, turn);
		headingTarget = heading;
		drivePower = turn;
		driveMode = AutoDriveMode.TURNING;
		timeout = timeoutS;
	}

	// Sweeps the robot according to turn parameter.
	void sweepRobot(double heading, double turn, double timeoutS) {
		resetEncoders();
		if (turn > 0) {
			moveWheels(turn, turn / 4);
		} else {
			moveWheels(Math.abs(turn) / 4, Math.abs(turn));
		}
		headingTarget = heading;
		drivePower = turn;
		driveMode = AutoDriveMode.SWEEPING;
		timeout = timeoutS;
	}

	// Stops the robot.
	void stopRobot() {
		resetEncoders();
		moveWheels(0, 0);
		driveMode = AutoDriveMode.STOPPED;
		timeout = 0;
	}

	// Apply low level drive signals
	void moveRobot(double fwd, double right) {
		right = clip(right, -MAX_TURN_RATE, MAX_TURN_RATE);
		moveWheels((fwd + right), (fwd - right));
	}

	// Apply low level drive signals
	void moveWheels(double left, double right) {
		lDriveM.set(clip(left, -1, 1));
		rDriveM.set(clip(right, -1, 1));
		robotIsMoving = ((left != 0) || (right != 0));
	}

	boolean moveIsComplete() {
		boolean complete = true;
		double distance = inchesTraveled();
		double distanceError = distanceTarget - distance;
		double power = drivePower;

		if (robotIsMoving && (timer.get() < timeout)) {
			switch (driveMode) {
			case STOPPED:
			default:
				break;

			case DRIVING:
				moveRobot(power, headingError * HEADING_DRIVE_GAIN);
				complete = (distance >= distanceTarget);
				break;

			case DRIVING_BRAKE:
				moveRobot(clip(distanceError * DRIVE_BRAKE_GAIN, 0, 1) * power, headingError * HEADING_DRIVE_GAIN);
				complete = (distance >= distanceTarget);
				break;

			case TURNING:
				power = clip(headingError * HEADING_TURN_GAIN, -1, 1) * power;
				moveRobot(0, power);
				complete = ((Math.abs(headingTarget - robotHeading)) <= HEADING_MATCH);
				break;

			case SWEEPING:
				power = clip(headingError * HEADING_TURN_GAIN, -1, 1) * power;
				if (power > 0) {
					moveWheels(power, power / 4);
				} else {
					moveWheels(Math.abs(power) / 4, Math.abs(power));
				}
				complete = ((Math.abs(headingTarget - robotHeading)) <= HEADING_MATCH);
				break;
			}
		}
		return complete;
	}

	// Run a lift control cycle and look for IN position condition
	void runLiftControl() {
		double error = liftSetPoint - liftHeight;
		double power;

		// Determine direction of required force
		if (error > 0) {
			power = (error * curLiftGain) + GRAVITY_LIFT;
		}
		else {
			power = (error * curLiftGain) / 3.0; // reduce down power
		}

		// Make sure the limit switches prevent excess motion
		if ((power > 0) && topLimit.get()) {
			power = GRAVITY_HOLD;
		} else if ((power < 0) && bottomLimit.get()) {
			power = 0;
		} else if (Math.abs(error) < 0.5) {
			// Apply enough force to hold lift in place
			power = GRAVITY_HOLD;
		}

		// Determine if lift is "inPosition" with a wider range
		liftInPosition = (Math.abs(error) <= 1.0) ;
		
		// Make sure power to lift is off if idle on bottom
		if (liftInPosition && bottomLimit.get()) {
			power = 0;
		}
		
		// output required lift power
		liftM.set(power);

		SmartDashboard.putNumber("Lift Error", error);
	}

	// Set the new lift target position
	void setLiftHeight(double setPoint) {

		// Look to see if we need to stage the descent to the floor
		if ((setPoint < 1.0) && (liftHeight > LIFT_POS_DRIVE)) {
			setPoint = LIFT_POS_DRIVE;
			goingToFloor = true; // Checked in robotPeriodic()
		} else {
			goingToFloor = false;
		}

		liftSetPoint = clip(setPoint, 0, MAX_HEIGHT);

		runLiftControl();
	}

	// Reset both Encoders & timer
	void resetEncoders() {
		lastDistanceTraveled = -1; // pre-load with invalid value
		lEncoder.reset();
		rEncoder.reset();
		timer.reset();
	}

	void openMouth() {
		mouthS.set(DoubleSolenoid.Value.kForward); // Mouth open
	}

	void closeMouth() {
		mouthS.set(DoubleSolenoid.Value.kReverse); // Mouth close
	}

	void setCollector(double speed) {
		lCollectM.set(speed);
		rCollectM.set(speed);
	}

	boolean liftIsReady(double timeout) {
		return (liftInPosition || (timer.get() > timeout));
	}

	boolean timerExpired(double timeout) {
		return (timer.get() > 0.4);
	}

	double getHeadingError() {
		// Determine heading error
		if (trackingCube && target_valid) {
			headingError = target_x;
		} else {
			headingError = headingTarget - robotHeading;
		}
	
		return headingError;
	}
	
	public void initLIDAR (int channel) {
		lidar = new Counter(channel);
		lidar.setMaxPeriod(1.0);
	    // Configure for measuring rising to falling pulses
		lidar.setSemiPeriodMode(true);
		lidar.reset();
	}
	
	/**
	 * Take a measurement and return the distance in cm
	 * 
	 * @return Distance in inches
	 */
	public double getRangeLIDAR() {
		double inch;
		
		/* If we haven't seen the first rising to falling pulse, then we have no measurement.
		 * This happens when there is no LIDAR-Lite plugged in, btw.
		 */
		if (lidar.get() > 1) {
			/* getPeriod returns time in seconds. The hardware resolution is microseconds.
			 * The LIDAR-Lite unit sends a high signal for 1 microseconds per mcm of distance.
			 */
			inch = (lidar.getPeriod() * 39370);
			return inch;

		}
		else {
			return 0;
		}
	}


}
