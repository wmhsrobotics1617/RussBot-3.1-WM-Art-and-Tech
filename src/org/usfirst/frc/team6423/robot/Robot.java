/*----------------------------------------------------------------------------
 * This is the 2018 Ward  Team 6423 robot code.
 * Disclaimer: The general code structure is a BAD example of proper coding practices but done for the following reasons:
 *  -The team has not made the time commitment to learn and use version control such as  hub,
 *    instead this single file with no folder structure is passed around on a memory stick or posted to google drive and merged by hand.
 *  -All variable and constant definitions are placed at the top of the code so there is one place to find everything. 
 *  -All functions are pasted in to this single file for the same reasons.
*/

package org.usfirst.frc.team6423.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import org.usfirst.frc.team6423.robot.subsystems.ExampleSubsystem;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Servo;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
//	private static final String kDefaultAuto = "Default";
//	private static final String kCustomAuto = "My Auto";
//	private String m_autoSelected;

	
	String text = "Hello";
	//Files.write("./Output.txt",text.getBytes());
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	//create speed controller objects on PWM channels											
	VictorSP driveMotorLeft1 	= new VictorSP(0);	//CIM 
	VictorSP driveMotorLeft2 	= new VictorSP(1);	//CIM
	VictorSP driveMotorRight1 	= new VictorSP(2);	//CIM
	VictorSP driveMotorRight2 	= new VictorSP(3);	//CIM
	Spark	 elevatorMotorLeft 	= new Spark(4);		//CIM
	Spark	 elevatorMotorRight	= new Spark(5);		//CIM
	Spark 	 intakeMotorLeft 	= new Spark(6);		//mini CIM
	Spark 	 intakeMotorRight 	= new Spark(7);		//mini CIM
	Talon 	 intakeMotorTilt 	= new Talon(8);		//redline 775
	Servo 	 drumPawlServo		= new Servo(9);		//servo that releases elevator drum pawl
		
	//fudge auto to drive straight
	private static final double LEFT_FUDGE  				= 1.1;//1.1 was good for 2nd robot
	private static final double RIGHT_FUDGE  				= 1.0;
		
	
	//define power panel connections so we can read current for each motor
	private static final int 	PWR_CHANNEL_DRIVE_MOTOR_LEFT1 	= 12; 	//40A circuit breaker
	private static final int 	PWR_CHANNEL_DRIVE_MOTOR_LEFT2  	= 13; 	//40A circuit breaker
	private static final int 	PWR_CHANNEL_DRIVE_MOTOR_RIGHT1  = 2; 	//40A circuit breaker
	private static final int 	PWR_CHANNEL_DRIVE_MOTOR_RIGHT2  = 3; 	//40A circuit breaker
	private static final int    PWR_CHANNEL_ELEVATOR_MOTOR_LEFT	= 1; 	//40A circuit breaker
	private static final int    PWR_CHANNEL_ELEVATOR_MOTOR_RIGHT= 14; 	//40A circuit breaker
	private static final int    PWR_CHANNEL_INTAKE_MOTOR_LEFT 	= 0; 	//40A circuit breaker
	private static final int    PWR_CHANNEL_INTAKE_MOTOR_RIGHT 	= 15;	//40A circuit breaker
	private static final int    PWR_CHANNEL_INTAKE_MOTOR_TILT 	= 4; 	//30A circuit breaker
	
	//define power panel current trip levels, Amps -  
	//Warning: redline motors burn out when stalled for long periods - Be careful when adjusting trip/restore points
	//Set trip levels below measured trip currents with ample margin especially redline 775.
	// measured stall current: Mini CIM  		28A 
	// measured stall current: Redline 775 		25A
	// measured stall current: CIM				TODO	
	//Why these stall currents reported by the power distribution board are much lower than the specs is
	// a question to be answered when time permits.
	//Regardless we need to use the measured values not the specs. 
	private static final double	TRIP_LEVEL_DRIVE_MOTORS 	 = 50;	//CIM
	private static final double TRIP_LEVEL_ELEVATOR_MOTORS	 = 125;	//CIM
	private static final double TRIP_LEVEL_INTAKE_MOTORS 	 = 23; 	//mini CIM
	private static final double TRIP_LEVEL_INTAKE_MOTOR_TILT = 20;	//changed to silver motor similar to redline 775 
	//define trip periods (how long over trip level before tripping), mS 
	//This is an allowance for the fact that when 1st powering motors they are effectively stalled until moving.
	private static final int TRIP_PERIOD_DRIVE_MOTORS 	  = 5000;
	private static final int TRIP_PERIOD_ELEVATOR_MOTORS  = 1000;
	private static final int TRIP_PERIOD_INTAKE_MOTORS 	  = 3000; 
	private static final int TRIP_PERIOD_INTAKE_MOTOR_TILT= 500;//keep redline short trip period 
	//define sustain periods (how long to stay tripped), mS 
	//This value should be considerable larger than the trip period for the red line motors so the average is not so high as to burn up the motor.
	//The CIMs can be stalled for long periods without burning up.
	private static final int TRIP_SUSTAIN_DRIVE_MOTORS 	  	= 5000;
	private static final int TRIP_SUSTAIN_ELEVATOR_MOTORS  	= 2000;
	private static final int TRIP_SUSTAIN_INTAKE_MOTORS		= 5000; 
	private static final int TRIP_SUSTAIN_INTAKE_MOTOR_TILT	= 3000;
	//create trip flags
	private static boolean TRIPPED_DRIVE_MOTORS 	= false;
	private static boolean TRIPPED_ELEVATOR_MOTORS  = false;
	private static boolean TRIPPED_INTAKE_MOTORS 	= false; 
	private static boolean TRIPPED_INTAKE_MOTOR_TILT= false;
	//create variables to count up to periods
	private static int TRIP_COUNT_DRIVE_MOTORS 		= 0;
	private static int TRIP_COUNT_ELEVATOR_MOTORS  	= 0;
	private static int TRIP_COUNT_INTAKE_MOTORS 	= 0; 
	private static int TRIP_COUNT_INTAKE_MOTOR_TILT	= 0;
	//define period processCurrentLimits() will be called
	private  static final int TRIP_PROCESS_CALL_PERIOD = 200; //call processCurrentLimits every 200mS
	//define how often, in mS  periodic functions are called, mS
	private  static final int TELEOP_CALL_PERIOD	= 20;
	private  static final int AUTO_CALL_PERIOD		= 20;
	//define how long in mS to drive motors to get across line during auto. This is only until nanoNav is working.
	private  static final int AUTO_WAIT_FOR_NAV_PERIOD	= 2000;//wait up to 1000mS  for nav to send commnads
	private  static final int AUTO_DRIVE_PERIOD			= 2500;//then drive 3000mS to cross line
	boolean nanoNavRespondedWithValidDriveForwardCommand= false; //used for logic to cross line if nav does not send commands
	boolean nanoNavLockedOut = false;	//this flag is set if the last flag is not set after 1000mS of auto period

	
	//define MAX motor drives 
	private static final double MOTOR_MAX_DRIVE_FORWARD   	= 0.4;
	private static final double MOTOR_MAX_DRIVE_REVERSE   	= 0.4;
	private static final double MOTOR_MAX_DRIVE_AUTO		= 0.6;
	private static final double MOTOR_MAX_DRIVE_AUTO_NAV	= 0.7;
	private static final double MOTOR_MAX_ELEVATOR_UP   	= 0.7;
	private static final double MOTOR_MAX_ELEVATOR_DOWN   	= 1.0;// 0.7 tested for normal elv - 1 needed for climb 250lbs
	private static final double MOTOR_MAX_INTAKE_PULL  		= 0.8;
	private static final double MOTOR_MAX_INTAKE_PUSH   	= 1.0;
	private static final double MOTOR_MAX_INTAKE_TILT_UP  	= 0;
	private static final double MOTOR_MAX_INTAKE_TILT_DOWN  = 0;
	
	//prevent face plane allowance
	private static final double FACE_PLANT_UP_ALLOWANCE     = 0.30;//how fast when elevator is full up, percent div by 100
	
	//create joy stick objects
	Joystick joyLeft  = new Joystick(0);  //move joy stick positions as needed in the driver station.. or change here
	Joystick joyRight = new Joystick(1);
	private static final int JOY_LEFT_DRIVE_AXIS  				= 1; //left joy, push/pull stick
	private static final int JOY_RIGHT_DRIVE_AXIS 				= 1; //right joy, push/pull stick
	private static final int JOY_RIGHT_INTAKE_PULL_BUTTON 		= 3; //left joy, roll thumb left
	private static final int JOY_RIGHT_INTAKE_PUSH_BUTTON 		= 4; //left joy, roll thumb right
	private static final int JOY_LEFT_INTAKE_TILT_UP_BUTTON 	= 3; //right joy, roll thumb left
	private static final int JOY_LEFT_INTAKE_TILT_DOWN_BUTTON 	= 4; //right joy, roll thumb right
	private static final int JOY_LEFT_ELEVATOR_UP_BUTTON 		= 1; //left joy, trigger with index finger
	private static final int JOY_RIGHT_ELEVATOR_DOWN_BUTTON 	= 1; //right joy, trigger with index finger
	private static final int JOY_RIGHT_SERVO_PAWL_RELEASE_ONE 	= 7; //left joy button on base, need to hold both
	private static final int JOY_RIGHT_SERVO_PAWL_RELEASE_TWO 	= 8; //left joy button on base, need to hold both
	private static final int JOY_LEFT_ELEVATOR_FULL_DOWN_BUTTON 	= 5; 
	private static final int JOY_LEFT_ELEVATOR_SWITCH_HEIGHT_BUTTON = 6; 
	private static final int JOY_RIGHT_ELEVATOR_SCALE_HEIGHT_BUTTON = 5; 
	private static final int JOY_RIGHT_ELEVATOR_BAR_HEIGHT_BUTTON 	= 6; 

	//---- create even more stuff
	PowerDistributionPanel powerPanel = new PowerDistributionPanel();
	SerialPort sp = new SerialPort(38400, SerialPort.Port.kMXP);
	private static final int RING_BUF_SIZE = 300;
	byte[] buf = new byte[100];
	byte[] ringBuf = new byte[RING_BUF_SIZE];
	int iRingPtr = 0;
	
	int iTimerCount = 0;
	DriverStation station;
	
	//---- Gyro related
	private  boolean autoDriveWithGyroCompensation = false;//set to true to use gyro during auto
	//ADXRS450_Gyro gyro; 
	//Port gyroPort;
	private static final double gyroGain = 0.03;
	double gyroDesiredHeading = 0;
	double previousLeftDrive = 0;	//used to reset heading each time nav sends new drive command after rotation
	double previousRightDrive = 0;
	
	//*************** stuff related to elevator control ***************************
	//The PID functionality of the encoder object will not be used as it would not be easy to debug or customize
	//So, the PID functionality will be implemented on its own with the encoder class supplying the position/rate info0	Encoder elevatorEncoder = new Encoder(0,1,false);//DIO 0 and 1, don't reverse
	Encoder elevatorEncoder = new Encoder(0,1,false);//DIO 0 and 1, don't reverse
	boolean elevatorEncoderValid = false;// set true when low limit switch is pressed and encoder is reset
	double elevatorTargetPosition = 0; 						//0 is down, 7600 is up - elevator should be down when battery connected
	boolean elevatorReachedTarget = false;					//used for single button press and hold button logic
	DigitalInput elevatorNotLimitDown = new DigitalInput(2); //limit switch closes when elevator is down
	private static final int ELEVATOR_FULL_UP_ENCODER_VALUE 	  = 8500;//7600 clears climb bar by a few inches and leaves about 2 inches for pid overshoot
	private static final int ELEVATOR_BAR_HEIGHT_ENCODER_VALUE 	  = 7800;//
	private static final int ELEVATOR_SCALE_HEIGHT_ENCODER_VALUE  = 8500;//TODO determine by trial and error
	private static final int ELEVATOR_SWITCH_HEIGHT_ENCODER_VALUE = 2700;//TODO determine by trial and error
	private static final int ELEVATOR_FULL_DOWN_ENCODER_VALUE  	  = 0;   //full down

	private static final int ELEVATOR_POSITION_TOLERANCE = 25;		//tolerance to prevent hunting around target, encoder counts
	//---- elevator PID stuff - put this in a class when there is more time
	private  static final boolean useElevatorPID = false;
	private  static final double pidP_Gain = 1; 	// P gain
	private  static final double pidI_Gain = 0; 	// I gain
	private  static final double pidD_Gain = 0; 	// D gain
	double pidI_State; 	// I state
	private static final double pidI_Max = 1.0; 	// I max - limits I wind up
	private static final double pidI_Min = -1.0; 	// I min - limits I wind up
	double pidD_State; 	// D state : Last position input

	//*************** stuff related to intake tilt control ***************************
	DigitalInput tiltLimitDown = new DigitalInput(3); 	//limit switch closes when tilt is down
	DigitalInput tiltLimitUp = new DigitalInput(4); 	//limit switch closes when tilt is up
	boolean tiltTargetPositionUp = true; 				//false is down, true is up - tilt should be up when battery connected
	//*************** stuff related to intake control ***************************
	int intakeTargetState 	= 0; 	//0 is off, 1 is pull in cube, 2 is push out cube	
	//*************** stuff related to drum lock servo pawl release ***************************
	boolean drumPawlReleased = false; // don't allow elevator up after pawl is released until after power cycle	

	//stuff
	String gameData = "not set yet";
	String lastPrint2ReduceChatter = "not set yet";
	int autoPeriodicCount = 0;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		SmartDashboard.putData("Auto mode", m_chooser);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		//placeholder until after we see how well cal works for each speed controller
		driveMotorLeft1.enableDeadbandElimination(false); 
		driveMotorLeft2.enableDeadbandElimination(false);
		driveMotorRight1.enableDeadbandElimination(false);
		driveMotorRight2.enableDeadbandElimination(false);
		elevatorMotorLeft.enableDeadbandElimination(false);
		elevatorMotorRight.enableDeadbandElimination(false);//don't eliminate dead band so PID has full control
		intakeMotorLeft.enableDeadbandElimination(false);
		intakeMotorRight.enableDeadbandElimination(false);
		intakeMotorTilt.enableDeadbandElimination(false);
		driveMotorLeft1.setBounds(2.0, 1.7, 1.5, 1.3, 1.0); // max, deadbandMax, center, deadbandMin, min
		driveMotorLeft2.setBounds(2.0, 1.7, 1.5, 1.3, 1.0); 
		driveMotorRight1.setBounds(2.0, 1.7, 1.5, 1.3, 1.0); //wide band on drive because joystick is sloppy
		driveMotorRight2.setBounds(2.0, 1.7, 1.5, 1.3, 1.0); 
		elevatorMotorLeft.setBounds(2.0, 1.51, 1.5, 1.49, 1.0); 
		elevatorMotorRight.setBounds(2.0, 1.51, 1.5, 1.49, 1.0); 
		intakeMotorLeft.setBounds(2.0, 1.51, 1.5, 1.49, 1.0); 
		intakeMotorRight.setBounds(2.0, 1.52, 1.5, 1.49, 1.0); 
		intakeMotorTilt.setBounds(2.0, 1.52, 1.5, 1.49, 1.0); 
			
		drumPawlServo.set(0.5);	//initial 1.5mS is center position which will be the case before match to hold pawl from releasing
		
	//	gyroPort = Port.kOnboardCS0;
	//	gyro = new ADXRS450_Gyro(gyroPort);	
	//	gyro.calibrate();
		LiveWindow.disableTelemetry(powerPanel);
		initVariablesForNewTestPeriod();
	}
	
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();
		drumPawlServo.set(0.5);	//initial 1.5mS is center position which will be the case before match to hold pawl from releasing
		
		initVariablesForNewTestPeriod();
		gyroDesiredHeading =0;// gyro.getAngle();
		
		//TODO: replace switches on NanoNav with commands from dashboard as explained in comments above
		gameData = DriverStation.getInstance().getGameSpecificMessage();

//this is how to access individual characters
//		if(gameData.charAt(2) == 'L')
//		{
		//	System.out.println("3rd char is L");
//		}
		
		//Nano code processes byte by byte so back to back writes by roboRio should be OK.
		//sp.writeString(gameData +"\r");
		//System.out.println("****** " + gameData + " *****  sent game data on sp");
		
		//Start Nav
		//sp.writeString("NAV\r");
		//System.out.println("sent NAV on sp");
		
		int iByteCount = sp.getBytesReceived();
		if(iByteCount > 0) 
		{
			sp.read(iByteCount);//flush
			
		}
				
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		
	    iTimerCount++;
	    processElevatorDrive();
	    processTilt();
	    processIntake();
	
	    if(iTimerCount == 2 || iTimerCount == 4|| iTimerCount == 6)
	    {
	    	sp.writeString(gameData +"\r");
			System.out.println("****** " + gameData + " *****  sent game data on sp");
	    }
	    if(iTimerCount == 10)
	    {
	    	//Start Nav
			sp.writeString("NAV\r");
			System.out.println("sent NAV on sp");
	    }
	    
	    
		if(iTimerCount%(TRIP_PROCESS_CALL_PERIOD/AUTO_CALL_PERIOD) == 0) //process the current limits every 200mS
		{
			processCurrentLimits();//set or clear the trip flags -=- -Don't call to often as it adds traffic to CAN bus
		}

	    //Give the nanoNav 1000mS to start sending commands. If it does not, cross the line without it
	    // by driving straight for 3 seconds
		
		if(!nanoNavRespondedWithValidDriveForwardCommand || nanoNavLockedOut)
		{
			if(iTimerCount == AUTO_WAIT_FOR_NAV_PERIOD/AUTO_CALL_PERIOD)
			{
				System.out.printf("** ERROR ** no valid nanoNav command received - driving across line\n");				
			}
			
			if(iTimerCount >= AUTO_WAIT_FOR_NAV_PERIOD/AUTO_CALL_PERIOD && iTimerCount <= (AUTO_DRIVE_PERIOD+AUTO_WAIT_FOR_NAV_PERIOD)/AUTO_CALL_PERIOD)
			{
				nanoNavLockedOut = true;//don't allow nav to command after we started driving without it
				driveMotorLeft1.set(MOTOR_MAX_DRIVE_AUTO);
				driveMotorLeft2.set(MOTOR_MAX_DRIVE_AUTO);
				driveMotorRight1.set(-MOTOR_MAX_DRIVE_AUTO);
				driveMotorRight2.set(-MOTOR_MAX_DRIVE_AUTO);
			}
			else 
			{
			    driveMotorLeft1.set(0);//hold position until after the auto period ends
				driveMotorLeft2.set(0);
				driveMotorRight1.set(0);
				driveMotorRight2.set(0);
			}
		}
	    
	    try
		{		
			//Problem: If we execute getBytesReceived in the middle of a transmission, we will read the bytes then 20mS later the rest of the transmission will be read.
	    	// If the rest of the transmission is less than 14 bytes, it will not be read until the sender sends another packet which will not pass the L && R test.
	    	// That packet would be discarded and any action in it lost.
	    	// If we discard the remaining bytes from the 1st transmission so the next one would be received, any action in the 1st would be lost.
	    	// The logs show 1 failure in ~16 transmissions and also 13 straight failures when the transmitter and receiver periods walk across each other
	    	//Correct solutions involve checksums and/or writing to a buffer and checking for the \r and/or \n but dealing with \r and \n 
	    	// but getting it to work with Putty, Visual studio and Java given that \r\n may vary  between them will be time consuming with limited development/test time on the hardware.
	    	// We need to weigh the likelihood of it working well with limited test time with doing it 100% right.
	    	//Possible Solution 1: 
	    	// -Increase 14 byte test to a value closer to the min expected (~45) to  reduce the occurrence of the issue.
	    	// -To allow tolerance for missed actions, have sender lock in the action
	    	// -and only perform the action when the action changes.
	    	// -change transmit period or receive period so at most one transmission is lost (wont walk over each other)
	    	//Possible Solution 2: (*** Implemented this solution** )
	    	// -Write received bytes to a ring buffer.
	    	// -After each byte is written and the ring pointer advanced, look back in ring buffer 14 bytes for the 'L' and 8 bytes for the 'R'
	    	// -If L and R are in those positions, parse and execute string from -14 and -1
	    	// -If is a good idea to have sender lock in action and only perform action when it changes. While Solution 2 eliminates lost actions due the issue seen
	    	// - data corruption of the single action command is possible given the noisy environment from induced EMF of high current motors. 
	    	int iByteCount = sp.getBytesReceived();
			if(iByteCount > 0) 
			{
				buf = sp.read(iByteCount);
				copyToRingBufAndProcess(iByteCount);
			}
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
	}
	
	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		drumPawlServo.set(0.5);	//initial 1.5mS is center position which will be the case before match to hold pawl from releasing

		initVariablesForNewTestPeriod();
		gyroDesiredHeading = 0;//gyro.getAngle();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() { //20mS period typical
		Scheduler.getInstance().run();
		
	    processElevatorDrive();
	    processTilt();
	    processIntake();

		iTimerCount++;
		if(iTimerCount%(TRIP_PROCESS_CALL_PERIOD/TELEOP_CALL_PERIOD) == 0) //process the current limits every 200mS
		{
		 	processCurrentLimits();//set or clear the trip flags -=- -Don't call to often as it adds traffic to CAN bus
		}
		
		if(iTimerCount%5 == 0)//every 100mS do stuff
		{					
			linkDriveMotorsToJoySticks();
			linkElevatorMotorsToTriggerButtons();
			linkIntakeMotorsToLeftJoyButtons();
			linkIntakeTiltMotorToRightJoyButtons();
			linkLockPawWithServoToJoystickButtons();
			linkElevatorTargetsToJoystickButtons();
		}
		
		if(iTimerCount%100 == 0)//every 2 seconds do stuff 
		{
		//	SmartDashboard.putNumber("Current_value", powerPanel.getTempe);
			
	
			//------- test print to smart dash board message window
//			SmartDashboard.putString("1st string", "2nd string");
//			SmartDashboard.putNumber("timer count ", iTimerCount);
			
			//------- test print to console
		//	System.out.printf("count %d\n", iTimerCount);
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}

	//This function loads a circular buffer from the buffer received from the nav module.
	//After each byte is written and the pointer is still pointing to that byte, a function is called to look back for the signature L and R in the correct positions.
	//If the L and R are found, the 14 byte command is executed.
	//The ring function is not disturbed by detecting and executing the command.
	private void copyToRingBufAndProcess(int iByteCount)
	{
		for(int i = 0; i < iByteCount; i++)
		{
			ringBuf[iRingPtr] = buf[i];
			lookBackForValidCommandAndProcess();//Pointer is now pointing to last byte written.
			iRingPtr++;
			if(iRingPtr >= RING_BUF_SIZE)
			{
				iRingPtr=0;
			}
		}	
	}

	// -Look back in ring buffer 13 bytes for the 'L' and 7 bytes for the 'R'
	// -If L and R are in those positions, parse and execute string from -14 and -1
	private void lookBackForValidCommandAndProcess() 
	{
		
		if((lookBackInBuffer(13) == 'L') && lookBackInBuffer(7) == 'R')
		{
			int iRingPtrTemp = 0;
			byte[] localBufCopy = new byte[14];
			for(int i = 0; i <=13; i++) 
			{
				iRingPtrTemp = iRingPtr + i - 13;
				if(iRingPtrTemp<0)
				{
					iRingPtrTemp += RING_BUF_SIZE;
				}
			//	System.out.println("index value in the ring buffer: "+ (iRingPtrTemp) );
			//	System.out.println("index value : "+ (i) );
				localBufCopy[i] = ringBuf[iRingPtrTemp];//copy bytes from 0 to 13 (14 bytes)
			}
			
			String s = new String(localBufCopy); //I assume this adds the null terminator
			if(lastPrint2ReduceChatter !=s)
			{
				System.out.println("RX: " + s);// debug statement will print out string
				lastPrint2ReduceChatter = s;			
			}
			
			//Example format L-008,R0055,A3,...
			//convert chars at index 1,2,3,4  to a left drive value between -100 and 100
			//convert chars at index 7,8,9,10 to a right drive value between -100 and 100
			//convert char at index 13 to an action value
		    double leftDriveSubs =0;
		    double  rightDriveSubs =0;
		    int action=0;
		    try
			{
			    String sLeftDrive = s.substring(1,5);
			    String sRightDrive = s.substring(7,11);
			 //   System.out.println("L drivestring  "+ sLeftDrive);
			 //   System.out.println("R drivestring  "+ sRightDrive);
			    double leftDrive = (Double.parseDouble(sLeftDrive));//100 converts from percent to 0 to 1.0
			    double rightDrive = (Double.parseDouble(sRightDrive));
			    leftDriveSubs = (leftDrive/100);
			    rightDriveSubs = (rightDrive/100);
			    action = localBufCopy[13] - '0'; //convert from ASCII char to int: digits are consecutive so we can do this
			}
			catch(Exception ex)
			{
				System.out.println("exception caught "+ ex.toString());
			}
		    //Disable auto drive across line feature when we know robot is driving forward from nav command
		    if(leftDriveSubs > 0 && leftDriveSubs == rightDriveSubs)
		    {
		    	nanoNavRespondedWithValidDriveForwardCommand = true;
		    }
		    if(previousLeftDrive != leftDriveSubs || previousRightDrive != rightDriveSubs)
		    {
		    	gyroDesiredHeading = 0;//gyro.getAngle();//set new heading each time nav sends new drive after rotation
		    	previousLeftDrive = leftDriveSubs;
		    	previousRightDrive = rightDriveSubs;
		    }
		    	
		    if(true)//(nanoNavRespondedWithValidDriveForwardCommand)
		    {
		    	driveWithGyroCompensation(leftDriveSubs, rightDriveSubs, autoDriveWithGyroCompensation);//false means don't comp	
		    	setActionRequest(action);
		    }		
		}
	}
	
	private byte lookBackInBuffer(int howFarBack)
	{
	//	System.out.println(" iRingPtr: " +iRingPtr);
	//	System.out.println(" howFarBack: " + howFarBack);
		
		
		if(iRingPtr >= howFarBack)
		{
			return ringBuf[iRingPtr - howFarBack];
		}
		else
		{	
			return ringBuf[iRingPtr + RING_BUF_SIZE - howFarBack];
		}
	}
	
	public void processCurrentLimits() //set or clear the trip flags  
	{	
		//---------------------------- process the 4 drive motors ------------------------------ 
		if(!TRIPPED_DRIVE_MOTORS) 
		//We need to be over the current limit on ANY ganged motor for the allowance period to set the tripped flag.
		{
			//Over the current limit ?
			if(powerPanel.getCurrent(PWR_CHANNEL_DRIVE_MOTOR_LEFT1)  > TRIP_LEVEL_DRIVE_MOTORS
			 ||powerPanel.getCurrent(PWR_CHANNEL_DRIVE_MOTOR_LEFT2)  > TRIP_LEVEL_DRIVE_MOTORS
			 ||powerPanel.getCurrent(PWR_CHANNEL_DRIVE_MOTOR_RIGHT1) > TRIP_LEVEL_DRIVE_MOTORS
			 ||powerPanel.getCurrent(PWR_CHANNEL_DRIVE_MOTOR_RIGHT2) > TRIP_LEVEL_DRIVE_MOTORS)
				
			{
				TRIP_COUNT_DRIVE_MOTORS++;
				//over the allowance period ?
				if(TRIP_COUNT_DRIVE_MOTORS > TRIP_PERIOD_DRIVE_MOTORS/TRIP_PROCESS_CALL_PERIOD)
				{
					TRIPPED_DRIVE_MOTORS = true; //trip 
					TRIP_COUNT_DRIVE_MOTORS = 0; //clear count so we can count trip sustain
					SmartDashboard.putString("**error: over current** ", "Drive Motors");
					System.out.println("**error: over current** Drive Motors");					
				}
			}
			else 
			{
				TRIP_COUNT_DRIVE_MOTORS = 0; //new lease on life so we don't accumulate transient bad samples 
			}
		}
		else
		//Wait for sustained period to clear the tripped flag. 
		{
			TRIP_COUNT_DRIVE_MOTORS++;
			if(TRIP_COUNT_DRIVE_MOTORS > TRIP_SUSTAIN_DRIVE_MOTORS/TRIP_PROCESS_CALL_PERIOD)
			{
				TRIPPED_DRIVE_MOTORS = false; //reset
				TRIP_COUNT_DRIVE_MOTORS = 0; //clear count so we can count trip period
			}
		}
		//---------------------------- process the 2 elevator motors ------------------------------ 
		if(!TRIPPED_ELEVATOR_MOTORS) 
		//We need to be over the current limit for ANY ganged motor for the allowance period to set the tripped flag.
		{
			//Over the current limit ?
			if(powerPanel.getCurrent(PWR_CHANNEL_ELEVATOR_MOTOR_LEFT)  > TRIP_LEVEL_ELEVATOR_MOTORS
			 ||powerPanel.getCurrent(PWR_CHANNEL_ELEVATOR_MOTOR_RIGHT) > TRIP_LEVEL_ELEVATOR_MOTORS)
			{
				TRIP_COUNT_ELEVATOR_MOTORS++;
				//over the allowance period ?
				if(TRIP_COUNT_ELEVATOR_MOTORS > TRIP_PERIOD_ELEVATOR_MOTORS/TRIP_PROCESS_CALL_PERIOD)
				{
					TRIPPED_ELEVATOR_MOTORS = true; //trip 
					TRIP_COUNT_ELEVATOR_MOTORS = 0; //clear count so we can count trip sustain
					SmartDashboard.putString("**error: over current** ", "Elevator Motors");
					System.out.println("**error: over current** Elevator Motors");
				}
			}
			else 
			{
				TRIP_COUNT_ELEVATOR_MOTORS = 0; //new lease on life so we don't accumulate transient bad samples 
			}
		}
		else
		//Wait for sustained period to clear the tripped flag. 
		{
			TRIP_COUNT_ELEVATOR_MOTORS++;
			if(TRIP_COUNT_ELEVATOR_MOTORS > TRIP_SUSTAIN_ELEVATOR_MOTORS/TRIP_PROCESS_CALL_PERIOD)
			{
				TRIPPED_ELEVATOR_MOTORS = false; //reset
				TRIP_COUNT_ELEVATOR_MOTORS = 0; //clear count so we can count trip period
			}
		}
		//---------------------------- process the 2 intake motors ------------------------------ 
		if(!TRIPPED_INTAKE_MOTORS) 
		//We need to be over the current limit for ANY ganged motor for the allowance period to set the tripped flag.
		{
			//Over the current limit ?
			if(powerPanel.getCurrent(PWR_CHANNEL_INTAKE_MOTOR_LEFT)  > TRIP_LEVEL_INTAKE_MOTORS
			 ||powerPanel.getCurrent(PWR_CHANNEL_INTAKE_MOTOR_RIGHT) > TRIP_LEVEL_INTAKE_MOTORS)
			{
				TRIP_COUNT_INTAKE_MOTORS++;
				//over the allowance period ?
				if(TRIP_COUNT_INTAKE_MOTORS > TRIP_PERIOD_INTAKE_MOTORS/TRIP_PROCESS_CALL_PERIOD)
				{
					TRIPPED_INTAKE_MOTORS = true; //trip 
					TRIP_COUNT_INTAKE_MOTORS = 0; //clear count so we can count trip sustain
					SmartDashboard.putString("**error: over current** ", "Intake Motors");
					System.out.println("**error: over current** Intake Motors");
				}
			}
			else 
			{
				TRIP_COUNT_INTAKE_MOTORS = 0; //new lease on life so we don't accumulate transient bad samples 
			}
		}
		else
		//Wait for sustained period to clear the tripped flag. 
		{
				TRIP_COUNT_INTAKE_MOTORS++;
				//over the sustain period ?
				if(TRIP_COUNT_INTAKE_MOTORS >(TRIP_SUSTAIN_INTAKE_MOTORS/TRIP_PROCESS_CALL_PERIOD))
				{
					TRIPPED_INTAKE_MOTORS = false; //reset
					TRIP_COUNT_INTAKE_MOTORS = 0; //clear count so we can count trip period
				}
		}
		//---------------------------- process the intake tilt motor ------------------------------ 
		if(!TRIPPED_INTAKE_MOTOR_TILT) 
		//We need to be over the current limit for the allowance period to set the tripped flag.
		{
			//Over the current limit ?
			if(powerPanel.getCurrent(PWR_CHANNEL_INTAKE_MOTOR_TILT)  > TRIP_LEVEL_INTAKE_MOTOR_TILT)
			{
				TRIP_COUNT_INTAKE_MOTOR_TILT++;
				//over the allowance period ?
				if(TRIP_COUNT_INTAKE_MOTOR_TILT > TRIP_PERIOD_INTAKE_MOTOR_TILT/TRIP_PROCESS_CALL_PERIOD)
				{
		//			TRIPPED_INTAKE_MOTOR_TILT = true; //trip 
					TRIP_COUNT_INTAKE_MOTOR_TILT = 0; //clear count so we can count trip sustain
					SmartDashboard.putString("**error: over current** ", "Intake Tilt Motor");
					System.out.println("**error: over current** Intake Tilt Motor");
				}
			}
			else 
			{
				TRIP_COUNT_INTAKE_MOTOR_TILT = 0; //new lease on life so we don't accumulate transient bad samples 
			}
		}
		else
		//Wait for sustained period to clear the tripped flag. 
		{
			TRIP_COUNT_INTAKE_MOTOR_TILT++;
			if(TRIP_COUNT_INTAKE_MOTOR_TILT > TRIP_SUSTAIN_INTAKE_MOTOR_TILT/TRIP_PROCESS_CALL_PERIOD)
			{
				TRIPPED_INTAKE_MOTOR_TILT = false; //reset
				TRIP_COUNT_INTAKE_MOTOR_TILT = 0; //clear count so we can count trip period
			}
		}
	}

	public void linkElevatorMotorsToTriggerButtons() 
	{
		if(joyLeft.getRawButton(JOY_LEFT_ELEVATOR_UP_BUTTON) && !TRIPPED_ELEVATOR_MOTORS)
		{
			elevatorTargetPosition = ELEVATOR_FULL_UP_ENCODER_VALUE; //drive up but don't lock in request
			elevatorReachedTarget = true; //cancel request by any of the 4 preset position button presses
		}
		else
		{
			if(joyRight.getRawButton(JOY_RIGHT_ELEVATOR_DOWN_BUTTON) && !TRIPPED_ELEVATOR_MOTORS)
			{
				elevatorTargetPosition = ELEVATOR_FULL_DOWN_ENCODER_VALUE; //drive down but don't lock in request
				elevatorReachedTarget = true; //cancel request by any of the 4 preset position button presses
			}
			else
			{
				if(elevatorReachedTarget)//stop only if no request by any of the 4 preset position button presses
				{
					elevatorTargetPosition = elevatorEncoder.get();//set target to current position i.e. stop 
				}
			}
		}
	}

	public void linkIntakeMotorsToLeftJoyButtons() 
	{
		//For now, spins intake one direction for each button
		//motors need to spin opposite directions because they are mounted as mirrors of each other
		//Implement other functions like toggle,latching as needed .
		if(joyRight.getRawButton(JOY_RIGHT_INTAKE_PULL_BUTTON) && !TRIPPED_INTAKE_MOTORS)
		{
			intakeTargetState = 1;				
		}
		else
		{
			if(joyRight.getRawButton(JOY_RIGHT_INTAKE_PUSH_BUTTON) && !TRIPPED_INTAKE_MOTORS)
			{
				intakeTargetState = 2;
			}
			else
			{
				intakeTargetState=0;
			}
		}
	}
	public void linkIntakeTiltMotorToRightJoyButtons() 
	{
		//For now, spins tilt motor one direction for each button
		if(joyLeft.getRawButton(JOY_LEFT_INTAKE_TILT_UP_BUTTON) && !TRIPPED_INTAKE_MOTOR_TILT)
		{
			tiltTargetPositionUp = true;	
			//System.out.println("tilt up request");
		}
		else
		{
			if(joyLeft.getRawButton(JOY_LEFT_INTAKE_TILT_DOWN_BUTTON) && !TRIPPED_INTAKE_MOTOR_TILT)
			{
				tiltTargetPositionUp = false;
				
				if(lastPrint2ReduceChatter != "tilt down")
				{
					lastPrint2ReduceChatter = "tilt down";
			//		System.out.println("tilt down request");
				}
			}
		}
	}

	public void linkDriveMotorsToJoySticks() 
	{
		//have drive motor follow joy sticks scaled by forward or reverse limit
		//The left and right motors are mirrors. Hence the negative sign for the left joy stick
		double leftDriveTarget = joyLeft.getRawAxis(JOY_LEFT_DRIVE_AXIS); 
		double rightDriveTarget = joyRight.getRawAxis(JOY_RIGHT_DRIVE_AXIS);
		
		//-------- prevent face plant: when elevator is down allow 100%; when up allow FACE_PLANT_UP_ALLOWANCE; linear between ---------
		//need to be careful not to change left right proportion when limiting - so scale both or neither	
		double elevatorLinearPositon = (double)elevatorEncoder.get()/(double)ELEVATOR_FULL_UP_ENCODER_VALUE;//probably don't need both casts
		double limit = FACE_PLANT_UP_ALLOWANCE + (1.0 - FACE_PLANT_UP_ALLOWANCE) * (1.0 - elevatorLinearPositon);//This is a bit more intuitive
		//double limit = 1 + elevatorLinearPositon * (upAllowance - 1);											 // than this after a bit of algebra.
		
		if(leftDriveTarget > limit || rightDriveTarget > limit ||
				leftDriveTarget < -limit || rightDriveTarget<-limit)//if either over limit
		{
			leftDriveTarget *= limit;	//scale both
			rightDriveTarget *= limit;	//-
		}
		//------- done face plant prevention code ------------------------------------------------------------------------------------
		
		if(!TRIPPED_DRIVE_MOTORS)
		{
			//left stick drives left motors
			if(leftDriveTarget >= 0)
			{
				//set max FORWARD
				driveMotorLeft1.set(-leftDriveTarget * MOTOR_MAX_DRIVE_FORWARD); //left motor CCW drives wheels CW so invert sign
				driveMotorLeft2.set(-leftDriveTarget * MOTOR_MAX_DRIVE_FORWARD);
			}
			else
			{
				//set max REVERSE
				driveMotorLeft1.set(-leftDriveTarget * MOTOR_MAX_DRIVE_REVERSE);
				driveMotorLeft2.set(-leftDriveTarget * MOTOR_MAX_DRIVE_REVERSE);				
			}
			//left stick drives left motors
			if(rightDriveTarget >= 0)
			{
				driveMotorRight1.set(rightDriveTarget * MOTOR_MAX_DRIVE_FORWARD);
				driveMotorRight2.set(rightDriveTarget * MOTOR_MAX_DRIVE_FORWARD);
			}
			else
			{
				driveMotorRight1.set(rightDriveTarget * MOTOR_MAX_DRIVE_REVERSE);
				driveMotorRight2.set(rightDriveTarget * MOTOR_MAX_DRIVE_REVERSE);				
			}	
		}
		else 
		{
			driveMotorLeft1.set(0);
			driveMotorLeft2.set(0);
			driveMotorRight1.set(0);
			driveMotorRight2.set(0);
		}
	}
	public void linkLockPawWithServoToJoystickButtons() 
	{
		if (joyRight.getRawButton(JOY_RIGHT_SERVO_PAWL_RELEASE_ONE) && joyRight.getRawButton(JOY_RIGHT_SERVO_PAWL_RELEASE_TWO) ) 
		{
			System.out.println("***** drum lock pawl released **********" );
			drumPawlReleased = true;	//prevent elevator up until after power cycle to prevent damage to drum stop assembly
			drumPawlServo.set(0);		// release pawl, 1.0ms. 
		}
		
	}
	public void linkElevatorTargetsToJoystickButtons() 
	{
		if (joyLeft.getRawButton(JOY_LEFT_ELEVATOR_FULL_DOWN_BUTTON)) 
		{
			System.out.println("***** elevator full down request **********" );
			elevatorTargetPosition = ELEVATOR_FULL_DOWN_ENCODER_VALUE;
			elevatorReachedTarget = false; //disable processing from setting drive to zero until we reach target
		}
		if (joyLeft.getRawButton(JOY_LEFT_ELEVATOR_SWITCH_HEIGHT_BUTTON))
		{
			System.out.println("***** elevator switch height request **********" );
			elevatorTargetPosition = ELEVATOR_SWITCH_HEIGHT_ENCODER_VALUE;
			elevatorReachedTarget = false; //disable processing from setting drive to zero until we reach target
		}
		if (joyRight.getRawButton(JOY_RIGHT_ELEVATOR_SCALE_HEIGHT_BUTTON)) 
		{
			System.out.println("***** elevator scale height request **********" );
			elevatorTargetPosition = ELEVATOR_SCALE_HEIGHT_ENCODER_VALUE;
			elevatorReachedTarget = false; //disable processing from setting drive to zero until we reach target
		}
		if (joyRight.getRawButton(JOY_RIGHT_ELEVATOR_BAR_HEIGHT_BUTTON)) 
		{
			System.out.println("***** elevator bar height request **********" );
			elevatorTargetPosition = ELEVATOR_BAR_HEIGHT_ENCODER_VALUE;
			elevatorReachedTarget = false; //disable processing from setting drive to zero until we reach target
		}
	}
		
	void processElevatorDrive()
	{
		//Is the limit switch pressed?
		if(!elevatorNotLimitDown.get())
		{
			elevatorEncoder.reset();	//yes:  reset the encoder to a known state
			elevatorEncoderValid = true;//      and enable normal elevator function
		}
		
		//If the encoder count was never reset to zero, allow the elevator to go down until the limit switch is pressed and checked next iteration
		if(!elevatorEncoderValid)
		{
			if(elevatorTargetPosition == 0)
			{
				elevatorMotorLeft.set(-MOTOR_MAX_ELEVATOR_DOWN);
				elevatorMotorRight.set(MOTOR_MAX_ELEVATOR_DOWN);
				System.out.println("Drivig elevator down only - waiting for limit switch");
			}
		}
		else //else normal elevator function
		{
			//System.out.println("normal elevator");
			if(useElevatorPID)	//drive elevator using PID
			{
				elevatorDriveUsingPID();
			}
			else		//drive elevator not using PID
			{
				if(elevatorTargetPosition > elevatorEncoder.get() + ELEVATOR_POSITION_TOLERANCE && !drumPawlReleased)
				{
					elevatorMotorLeft.set(MOTOR_MAX_ELEVATOR_UP);
					elevatorMotorRight.set(-MOTOR_MAX_ELEVATOR_UP);
				}
				else 
				{
					if(elevatorTargetPosition < elevatorEncoder.get() - ELEVATOR_POSITION_TOLERANCE)
					{
						elevatorMotorLeft.set(-MOTOR_MAX_ELEVATOR_DOWN);
						elevatorMotorRight.set(MOTOR_MAX_ELEVATOR_DOWN);
					}
					else 
					{
						elevatorMotorLeft.set(0);
						elevatorMotorRight.set(0);
						elevatorReachedTarget = true;
					}
				}
			}
		}
	}
	void elevatorDriveUsingPID()
	{
		double error = ( (elevatorTargetPosition - elevatorEncoder.get() )/ (ELEVATOR_FULL_UP_ENCODER_VALUE) );
		double pidDrive = updateElevatorPID( error,  elevatorEncoder.get());
		
		if(!elevatorNotLimitDown.get())//This may have been called previously but can't hurt to make sure
		{
			elevatorEncoder.reset();
			elevatorEncoderValid = true;
			resetPidState();
		}
		if(elevatorEncoderValid)//This flag is probably set as well but make sure
		{
			elevatorMotorLeft.set(pidDrive * MOTOR_MAX_ELEVATOR_DOWN);
			elevatorMotorRight.set(-pidDrive * MOTOR_MAX_ELEVATOR_DOWN);
		}
	}
	
	//*********************************** PID *********************************
	//refer to  "PID Without a PhD" by Tim Wescott - This is the same code hacked into java
	//The return value is capped at 1.0 and -1.0
	double updateElevatorPID(double error, double position)
	{
		double pTerm, dTerm, iTerm;
		//calculate P term
		pTerm = pidP_Gain * error; // calculate the proportional term
		
		// calculate the integral state with appropriate limiting
		pidI_State += error;
		
		// Limit the I state to avoid I wind up issue
		if (pidI_State > pidI_Max)
		{
			pidI_State = pidI_Max;
		}
		else if (pidI_State < pidI_Min)
		{
			pidI_State = pidI_Min;
		}
		
		// calculate I term
		iTerm = pidI_Gain * pidI_State;
		// calculate D term
		dTerm = pidD_Gain * (pidD_State - position);
		pidD_State = position;
		
		//calculate the answer and limit to +/- 1.0
		double dRet = pTerm + dTerm + iTerm;
		if(dRet > 1.0)
		{
			dRet = 1.0;
		}
		if(dRet < -1.0)
		{
			dRet = -1.0;
		}
		if(iTimerCount%1000 == 0)
		{
			System.out.println("pidD_State : "+pidD_State);
			System.out.println("pTerm : " +pTerm );
			System.out.println("iTerm : "+iTerm);
			System.out.println("dTerm : "+dTerm);
		}
		
		return dRet;
		
	}
	void resetPidState() 
	{
			pidD_State = 0;
			pidI_State = 0;
	}
	//*************************************************************************
	void processTilt()
	{
		if(tiltTargetPositionUp && tiltLimitUp.get())
		{
			intakeMotorTilt.set(-MOTOR_MAX_INTAKE_TILT_UP);
			//System.out.println("tilt up drive");
		}
		else
		{
			if(!tiltTargetPositionUp && tiltLimitDown.get())
			{
				intakeMotorTilt.set(MOTOR_MAX_INTAKE_TILT_DOWN);
				//System.out.println("tilt drive down");
				
			}//
			else 
			{
				intakeMotorTilt.set(0);
			}		
		}
	}
	void processIntake()
	{
		switch (intakeTargetState)
		{
			case 0:
			default:
				intakeMotorLeft.set(0);
				intakeMotorRight.set(0);
				break;
			case 1:
				intakeMotorLeft.set(MOTOR_MAX_INTAKE_PULL);
				intakeMotorRight.set(-MOTOR_MAX_INTAKE_PULL);
				break;
			case 2:
				intakeMotorLeft.set(-MOTOR_MAX_INTAKE_PUSH);
				intakeMotorRight.set(MOTOR_MAX_INTAKE_PUSH);
				break;		
		}
	}

	void setActionRequest(int action)
	{
		//Maddy and Josh needs to agree on what the actions are
		//Until then put in some example code
		switch (action)
		{
			case 0:
			default:
				//System.out.println("No action sent");
				break; //perform no action for these cases
			case 1:
				elevatorTargetPosition = 0;//full down position
				System.out.println("Reset to the bottom");
				break;
			case 2:
				elevatorTargetPosition = ELEVATOR_SCALE_HEIGHT_ENCODER_VALUE;//scale height
				System.out.println("Raise to scale");
				break;
			case 3:
				elevatorTargetPosition = ELEVATOR_SWITCH_HEIGHT_ENCODER_VALUE;//switch height
				System.out.println("Raise to switch");
				break;
			case 4:
				tiltTargetPositionUp = false; //tilt down
				System.out.println("lower claw down");
				break;
			case 5:
				tiltTargetPositionUp = true; //tilt up
				System.out.println("Raise claw up");
				break;
			case 6:
				intakeTargetState = 1;//pull in cube
				break;
			case 7:
				intakeTargetState = 2; //push out cube
				System.out.println("expel cube");
				break;
		}
	}
		
			
	public void driveWithGyroCompensation(double leftDrive, double rightDrive, boolean useGyroToCompensate) 
	{
	// We are comparing the current angle value to that of the original value. 
		
		if(leftDrive==rightDrive && useGyroToCompensate)
		{
			double current_value =0;// gyro.getAngle();
			double angeleCompensation  = current_value - gyroDesiredHeading;//calc error - pos angle is CCW
			double compLeftDrive = leftDrive + angeleCompensation * gyroGain;
			double compRightDrive = rightDrive + angeleCompensation * gyroGain;
			if(compLeftDrive>1.0)
			{
				compLeftDrive = 1.0;
			}
			if(compLeftDrive < -1.0)
			{
				compLeftDrive = -1.0;
			}

			if(compRightDrive > 1.0)
			{
				compRightDrive = 1.0;
			}

			if(compRightDrive < -1.0)
			{
				compRightDrive = -1.0;
			}
			driveMotorLeft1.set(compLeftDrive  * MOTOR_MAX_DRIVE_AUTO_NAV); //left motor CCW drives wheels CW so invert sign
			driveMotorLeft2.set(compLeftDrive  * MOTOR_MAX_DRIVE_AUTO_NAV);
			driveMotorRight1.set(-compRightDrive * MOTOR_MAX_DRIVE_AUTO_NAV);
			driveMotorRight2.set(-compRightDrive * MOTOR_MAX_DRIVE_AUTO_NAV);
		}
	    else 
	    {
			driveMotorLeft1.set(leftDrive  * MOTOR_MAX_DRIVE_AUTO_NAV * LEFT_FUDGE); //left motor CCW drives wheels CW so invert sign
			driveMotorLeft2.set(leftDrive  * MOTOR_MAX_DRIVE_AUTO_NAV *LEFT_FUDGE);
			driveMotorRight1.set(-rightDrive * MOTOR_MAX_DRIVE_AUTO_NAV * RIGHT_FUDGE);
			driveMotorRight2.set(-rightDrive * MOTOR_MAX_DRIVE_AUTO_NAV *RIGHT_FUDGE); 
		}
	}
	//Each time the enable button is pressed on the driver station, we need to reset these variables 
	void initVariablesForNewTestPeriod()
	{
		nanoNavRespondedWithValidDriveForwardCommand = false;
		nanoNavLockedOut = false;	
		iTimerCount = 0;
		gyroDesiredHeading = 0;
	}
}