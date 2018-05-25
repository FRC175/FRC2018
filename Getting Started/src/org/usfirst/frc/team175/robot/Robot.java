/*----------------------------------------------------------------------------*/
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

/**
 * @author FRC Team "Buzz" 175
 */

package org.usfirst.frc.team175.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

/*
 * Alert: All directions are in the POV of the robot
 *
 * Controls:
 *
 * Operator:
 * Left:
 * Stick) Elevator position, only when button 4 is held
 * Trigger) sucks in cube
 * 2)
 * 3)
 * 4) switches elevator control to joystick
 * 5) Grabber Rotate
 * 6) 
 * 7)Elevator position 1
 * 8)ELevator position 2
 * 9)Elevator position 3
 * 10)
 * 11) grabber release
 * 12)
 *
 * Driver:
 * Right:
 * Stick) Drive**
 * Trigger) Lateral Drive Deploy and operate
 * 2) shifting
 * 3)
 * 4)
 * 5)
 * 6)
 * 7) climber extends
 * 8) climber retracts
 * 9) alignment
 * 10)rotate
 * 11)toggles winch
 * 12) ** if trigger is activated then Stick now controls Lateral Drive
 */

public class Robot extends IterativeRobot {

    //================================================================================
    // Declarations & Definitions
    //================================================================================
    // Drive TalonSRXs------------WPI_TalonSRX(int CAN id)------No physical difference between the WPI_TalonSRX and TalonSRX
    private WPI_TalonSRX rightMaster = new WPI_TalonSRX(0); // RIGHT DRIVE MASTER
    private WPI_TalonSRX rightFollower = new WPI_TalonSRX(1); //RIGHT DRIVE FOLLOWER
    private WPI_TalonSRX leftMaster = new WPI_TalonSRX(2); // LEFT DRIVE MASTER
    private WPI_TalonSRX leftFollower = new WPI_TalonSRX(3); //LEFT DRIVE FOLLOWER

    // Manipulator TalonSRXs-------- TalonSRX(int CAN id)
    private TalonSRX elevator = new TalonSRX(4); // TalonSRX purposed for not yet assigned manipulator
    private TalonSRX lateralDrive = new TalonSRX(5); // TalonSRX purposed for lateral Drive

    // Manipulator TalonSRs----------Talon(int PWM i/o)
    private Talon winch = new Talon(0); // TalonSR purposed for not yet assigned manipulator
    private Talon grabRollerL = new Talon(1); // TalonSR purposed for left side of the rollers
    private Talon grabRollerR = new Talon(2); // TalonSR purposed for right side of the rollers
    private Talon donker = new Talon(3); // The TalonSR purposed for activating the belts within the one use autonomous mechanism
    private Talon climbExtendSR = new Talon(4); // Needs actual I/O

    //Servo------------------------------------------------
    private Servo autoLeftServo = new Servo(5); //servo for deploying the left auto limit switch
    private Servo autoRightServo = new Servo(6); //servo for deploying the right auto limit switch

    // Double Solenoids-------DoubleSolenoid(int CAN id of PCM, int forward channel, int reverse channel)
    private DoubleSolenoid climbRotate = new DoubleSolenoid(0, 0, 1); // solenoid purposed for controlling the rotation of the climbing mechanism

    //Single Solenoids--------------Solenoid(int CAN id of PCM, int channel)
    private Solenoid lateralDeploy = new Solenoid(0, 2); //solenoid purposed for deploying the lateral drive system
    private Solenoid shift = new Solenoid(0, 3); //solenoid purosed for shifting
    private Solenoid climbAlign = new Solenoid(0, 4); //solenoid purposed for deploying the climb hook delivery system CHANGE
    private Solenoid grabberRotatePN = new Solenoid(0, 5); //solenoid purposed for manipulating cube grabber position CHANGE

    // LimitSwitches----DigitalInput(int i/o)
    private DigitalInput autoLeftLimit = new DigitalInput(1); //limit switch to determine whether the  left bumper is against the switch in auto
    private DigitalInput autoRightLimit = new DigitalInput(2); //limit switch to determine whether the right bumper is against the switch in auto
    private DigitalInput climbDown = new DigitalInput(3); // Limit switch to determine if climb arm is retracted all the way down
    private DigitalInput climbUp = new DigitalInput(4); // Limit switch to determine if climb arm is extended all the way up
    private DigitalInput limitGrabber = new DigitalInput(9);

    // IR Sensors------DigitalInput(int i/o)
    //private DigitalInput irAutoRelease = new DigitalInput(9); //ir sensor to signal when the donker is aligned with the switch TESTING PURPOSES ONLY
    private DigitalInput irDonker = new DigitalInput(7);
    private DigitalInput irManipLeft = new DigitalInput(6);
    private DigitalInput irManipRight = new DigitalInput(5);
    //private DigitalInput irGrabber = new DigitalInput(0); //limit switch to determine whether a power cube is within the grabber

    // Relay---------Relay(int i/o)
    private Relay relayOne = new Relay(3);
    private Relay relayTwo = new Relay(2);
    private Relay relayThree = new Relay(1);
    private Relay grabberLight = new Relay(0);

    // Booleans
    private boolean grabRotateToggle = false; // boolean utilized within the grabber section of code; specifically utilized within the grabber rotation section L button 7
    private boolean deploy = false; // boolean that determines whether or not lateral is deployed
    private boolean grabRotatePress = true;
    private boolean alignPress = true; //boolean that ensures that the toggle for alignment actually toggles
    private boolean alignToggle = false; //boolean that toggles the alignment
    private boolean elevatorZone = false;
    private boolean shiftToggle = false;
    private boolean shiftPress = false;
    private boolean autoStart;
    private boolean timeStart = true;
    private boolean canElevatorBeRaised = false;
    private boolean grabOut = false;
    private boolean alignOut = false;

    // Double
    private double targetPositionRotations;
    private double wantedPositionRotations;
    private double angle;
    //private double switchSide;
    private double time;
    private final double DRIVE_TO_INCHES = 176;
    private final double LATERAL_TO_INCHES = 42.965;
    private final double rRatio = 1.0662;
    private final double lRatio = .917;

    // Integers
    private int autoStep;
    private int rightPosition;
    private int leftPosition;
    private int lateralPosition;
    private int elevatorPosition;
    private int autoType; // Change between different autonomous modes
    private int x = 0;

    // Constants
    private final int K_SLOT_INDEX = 0;
    private final int K_PID_LOOP_INDEX = 0;
    private final int K_TIMEOUT_MS = 10;
    private final int ELEVATOR_POSITION = elevator.getSelectedSensorPosition(K_TIMEOUT_MS) & 128;
    private final int LEFT_DRIVE_POSITION = 0; //leftMaster.getSelectedSensorPosition(K_TIMEOUT_MS) & 128;
    private final int RIGHT_DRIVE_POSITION = 0; //rightMaster.getSelectedSensorPosition(K_TIMEOUT_MS) & 128;
    private final int LATERAL_DRIVE_POSITION = 0;
   

    // String
    private String switchSide;
    private String scaleSide;
   
    // Enums
    private enum RobotType {
        ROBOT_ONE, ROBOT_TWO
    }

    // Gyro
    ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    // Other
    private DifferentialDrive RobotDrive = new DifferentialDrive(rightMaster, leftMaster); // In order to establish follower SRXs: "Name of SRX".set(ControlMode.Follower, "CAN id of master");
    private Joystick rightStick = new Joystick(0);
    private Joystick leftStick = new Joystick(1);
    private Timer autoTimer = new Timer();
    private SendableChooser<Integer> autoChooser;
    private SendableChooser<Double> kP_elevator;
    private SendableChooser<Double> kI_elevator;
    private SendableChooser<Double> kD_elevator;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        //================================================================================
        // Misc.
        //================================================================================
        // CameraServer.getInstance().startAutomaticCapture();
        /*new Thread(() -> {
            CameraServer.getInstance().startAutomaticCapture();
        }).start();*/
       
        elevatorZone = false;
       
        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);
        leftMaster.setInverted(false);
        rightMaster.setInverted(false);
        leftFollower.setInverted(false);
        rightFollower.setInverted(false);
       
        //================================================================================
        // Elevator PID Configuration
        //================================================================================
        elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
        elevator.setSensorPhase(false);
       
        elevator.configNominalOutputForward(0, K_TIMEOUT_MS);
        elevator.configNominalOutputReverse(0, K_TIMEOUT_MS);
        elevator.configPeakOutputForward(1, K_TIMEOUT_MS);
        elevator.configPeakOutputReverse(-1, K_TIMEOUT_MS);

        elevator.configAllowableClosedloopError(0, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
       
        //PIDController elevatorPID = new PIDController();
        //PIDSource
       

        elevator.config_kF(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);
        elevator.config_kP(K_PID_LOOP_INDEX, 1.0, K_TIMEOUT_MS);
        elevator.config_kI(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);
        elevator.config_kD(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);

        elevator.setSelectedSensorPosition(ELEVATOR_POSITION, K_PID_LOOP_INDEX, K_TIMEOUT_MS); //256 counts per rotation
       
        //current limiting
        elevator.configContinuousCurrentLimit(20, 0); //set current
        elevator.configPeakCurrentLimit(30, 0); //current limit
        elevator.configPeakCurrentDuration(100, 0); //how long you can pass the limit before pushed to default
        elevator.enableCurrentLimit(true); //on
       
        //Soft limit resticting elevator positioning
        elevator.configForwardSoftLimitThreshold(0, 0);
        elevator.configReverseSoftLimitThreshold(-30000, 0);
        elevator.configForwardSoftLimitEnable(false, 0); //off
        elevator.configReverseSoftLimitEnable(false, 0); //off
       
        elevator.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake); //brake
        //================================================================================
        // LeftMaster PID Configuration
        //================================================================================
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
        leftMaster.setSensorPhase(true);

        leftMaster.configNominalOutputForward(0, K_TIMEOUT_MS);
        leftMaster.configNominalOutputReverse(0, K_TIMEOUT_MS);
        leftMaster.configPeakOutputForward(1, K_TIMEOUT_MS);
        leftMaster.configPeakOutputReverse(-1, K_TIMEOUT_MS);

        leftMaster.configAllowableClosedloopError(0, K_PID_LOOP_INDEX, K_TIMEOUT_MS);

        leftMaster.config_kF(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);
        leftMaster.config_kP(K_PID_LOOP_INDEX, 0.12, K_TIMEOUT_MS);
        leftMaster.config_kI(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);
        leftMaster.config_kD(K_PID_LOOP_INDEX, 0.0012, K_TIMEOUT_MS);

        leftMaster.setSelectedSensorPosition(LEFT_DRIVE_POSITION, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
        leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake); //brake

        //================================================================================
        // RightMaster PID Configuration
        //================================================================================
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
        rightMaster.setSensorPhase(false);

        rightMaster.configNominalOutputForward(0, K_TIMEOUT_MS);
        rightMaster.configNominalOutputReverse(0, K_TIMEOUT_MS);
        rightMaster.configPeakOutputForward(1, K_TIMEOUT_MS);
        rightMaster.configPeakOutputReverse(-1, K_TIMEOUT_MS);

        rightMaster.configAllowableClosedloopError(0, K_PID_LOOP_INDEX, K_TIMEOUT_MS);

        rightMaster.config_kF(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);
        rightMaster.config_kP(K_PID_LOOP_INDEX, 0.08, K_TIMEOUT_MS);
        rightMaster.config_kI(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);
        rightMaster.config_kD(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);

        rightMaster.setSelectedSensorPosition(RIGHT_DRIVE_POSITION, K_PID_LOOP_INDEX, K_TIMEOUT_MS);

        rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake); //brake
        //================================================================================
        // LateralDrive PID Configuration
        //================================================================================
        lateralDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
        lateralDrive.setSensorPhase(true);

        lateralDrive.configNominalOutputForward(0, K_TIMEOUT_MS);
        lateralDrive.configNominalOutputReverse(0, K_TIMEOUT_MS);
        lateralDrive.configPeakOutputForward(1, K_TIMEOUT_MS);
        lateralDrive.configPeakOutputReverse(-1, K_TIMEOUT_MS);

        lateralDrive.configAllowableClosedloopError(0, K_PID_LOOP_INDEX, K_TIMEOUT_MS);

        lateralDrive.config_kF(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);
        lateralDrive.config_kP(K_PID_LOOP_INDEX, 10.0, K_TIMEOUT_MS);
        lateralDrive.config_kI(K_PID_LOOP_INDEX, 0.0, K_TIMEOUT_MS);
        lateralDrive.config_kD(K_PID_LOOP_INDEX, 2.0, K_TIMEOUT_MS);

        lateralDrive.setSelectedSensorPosition(LATERAL_DRIVE_POSITION, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
       
        lateralDrive.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake); //brake
       
        autoChooser = new SendableChooser<>();
       
        autoChooser.addDefault("Drive Straight", 0);
        autoChooser.addObject("Start on the LEFT, based on FMS SCALE, SWITCH or NONE (1)", 1);
        autoChooser.addObject("Start on the RIGHT, based on FMS SCALE, SWITCH or NONE (2)", 2);
        autoChooser.addObject("Start in the MIDDLE, load SWITCH based on FMS (6)", 6);
        autoChooser.addObject("UNTESTED Start at the LEFT; load SCALE based on FMS (4)", 4);
        autoChooser.addObject("UNTESTED Start at the LEFT; based on FMS SCLAE, SWITCH, or NONE, with double on scale (3)", 3);
        autoChooser.addObject("UNTESTED cross field double cube scale", 7);
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putBoolean("Do we have a cube?", limitGrabber.get());
       
        // For future elevator PID use
        /*double[] elevatorPID = new double[4];
       
        SmartDashboard.getNumberArray("Elevator PID", elevatorPID);
        SmartDashboard.putNumberArray("Elevator PID", elevatorPID);*/
       
   
    }

    /**
     * This function is run once each time the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() {
        zeroEncoders();
        gyro.reset();
        autoTimer.reset();
        autoTimer.start();
        angle = gyro.getAngle();
        double Kp = -.05;
       
        String gameData;
        try {
            gameData = DriverStation.getInstance().getGameSpecificMessage();
            switchSide = gameData.charAt(0) == 'L' ? "left" : "right";
            scaleSide = gameData.charAt(1) == 'L' ? "left" : "right";
        } catch(Exception e) {
            switchSide = "left";
            scaleSide = "left";
        }
       
   
        //autoType = (int) SmartDashboard.getNumber("DB/Slider 0", 0.0); // Set the auto type based on selected option on Driver Station
        autoType = autoChooser.getSelected();
        //autoType = 6;
        autoStep = 0;
        autoStart = true;
        rightPosition = rightMaster.getSelectedSensorPosition(RIGHT_DRIVE_POSITION);
        leftPosition = leftMaster.getSelectedSensorPosition(LEFT_DRIVE_POSITION);
       
       
        winch.set(0);
    }
   
    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
       
        /*
         * TODO: Drive forward grab game object lift with elevator, partway rotate 180 score depending on color of scale
         *
         * Just drive forward autonomous possibly score game object
         *
         * Streamline code
         */
       
       
        // 175.9 clicks per 1 inch
        // Store encoder readouts
        if(autoStart) {
            autoStep = 0;
            autoStart = false;
        }
        elevatorPosition = elevator.getSelectedSensorPosition(ELEVATOR_POSITION);
        rightPosition = rightMaster.getSelectedSensorPosition(RIGHT_DRIVE_POSITION);
        leftPosition = leftMaster.getSelectedSensorPosition(LEFT_DRIVE_POSITION);
        lateralPosition = lateralDrive.getSelectedSensorPosition(LATERAL_DRIVE_POSITION);
        angle = gyro.getAngle();

        switch (autoType) {
            case 0:
                if(autoStep == 0) {
                    rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    zeroEncoders();
                    gyro.reset();
                    autoStep++;
                }
                if(autoStep == 1) {
                    if(gyro.getAngle() > -5 && gyro.getAngle()< 5) {
                        if (rightPosition <= -100*DRIVE_TO_INCHES && leftPosition <= -100*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.75);
                        }
                } else {
                    gyroAdjust(0, 5, .75, false);
                }
                }
                if(autoStep == 2) {
                    autoStep++;
                }
                break;
            case 1: // TODO: start from the left and move straight
                if (autoStep == 0) {
                    rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    zeroEncoders();
                    gyro.reset();
                    autoStep++;
                }
                if(scaleSide.equals("left")) {
                    if(autoStep == 1) { //Drive straight
                        if(gyro.getAngle() > -5 && gyro.getAngle()< 5) {
                            if (rightPosition <= -280*DRIVE_TO_INCHES && leftPosition <= -280*DRIVE_TO_INCHES ) {
                                leftMaster.set(ControlMode.PercentOutput, 0);
                                rightMaster.set(ControlMode.PercentOutput, .0);
                                if(timeStart) {
                                    autoTimer.reset();
                                    timeStart = false;
                                }
                                if(autoTimer.get() > .5) {
                                    autoStep++;
                                    zeroEncoders();
                                }
                            } else {
                                leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                                rightMaster.set(ControlMode.PercentOutput, -.75);
                            }
                    } else {
                        gyroAdjust(0, 5, .75, false);
                    }
                }
                    if(autoStep == 2) { //raise elevator
                        if(elevatorPosition < -32050 ) {
                            autoStep++;
                        } else {
                            elevator.set(ControlMode.Position, -33050);
                        }
                    }
                    if(autoStep == 3) { //rotate 50 degrees toward scale
                        gyroAdjust(45, 5, .75, true);
                    }
                    if(autoStep == 4) { //spit out cube
                        if(!limitGrabber.get()) {
                            autoStep++;
                            grabRollerState(0, true);
                        } else {
                            grabRollerL.set(.4);
                            grabRollerR.set(.4);
                        }
                    }
                    if(autoStep == 5) {
                        if (rightPosition >= 12*DRIVE_TO_INCHES && leftPosition >= 12*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, -.50); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, .50);
                        }
                    }
                    if(autoStep == 6) {
                        elevator.set(ControlMode.Position, 0);
                        autoStep++;
                    }
                } else if(switchSide.equals("left")) {
                    if(autoStep == 1) { //Drive straight
                        if (rightPosition <= -145*DRIVE_TO_INCHES && leftPosition <= -145*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.75);
                        }
                    }
                    if(autoStep == 2) { //raise elevator
                        if(elevatorPosition < -11050 ) {
                            autoStep++;
                        } else {
                            elevator.set(ControlMode.Position, -12050);
                        }
                    }
                    if(autoStep == 3) { //rotate 90 degrees toward scale
                        gyroAdjust(70, 5, .75, true);
                    }
                    if(autoStep == 4) {
                        if (rightPosition <= -25*DRIVE_TO_INCHES && leftPosition <= -25*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                grabberRotatePN.set(true);
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.75);
                        }
                    }
                    if(autoStep == 5) { //spit out cube
                        grabRollerState(1, false);
                        autoStep++;
                    }
                }else {
                    if(autoStep == 1) { //Drive straight
                        if(gyro.getAngle() > -5 && gyro.getAngle() < 5) {
                            if (rightPosition <= -180*DRIVE_TO_INCHES && leftPosition <= -180*DRIVE_TO_INCHES ) {
                                leftMaster.set(ControlMode.PercentOutput, 0);
                                rightMaster.set(ControlMode.PercentOutput, .0);
                                if(timeStart) {
                                    autoTimer.reset();
                                    timeStart = false;
                                }
                                if(autoTimer.get() > .5) {
                                    autoStep++;
                                }
                            } else {
                                leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                                rightMaster.set(ControlMode.PercentOutput, -.75);
                            }
                        } else {
                            gyroAdjust(0, 5, .75, false);
                        }
                    }
                    if(autoStep == 2) { //raise elevator
                        autoStep++;
                    }
                    if(autoStep == 3) { //rotate 90 degrees toward scale
                        gyroAdjust(90, 1, .75, true);
                    }
                    if(autoStep == 4) { //spit out cube
                        autoStep++;
                    }
                }
                break;
            case 2: //TODO:start from the right and move straight, if scale right, put it in scale, if switch right put it in switch, otherwise move to left
                if (autoStep == 0) {
                    rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    zeroEncoders();
                    gyro.reset();
                    autoStep++;
                }
                if(scaleSide.equals("right")) {
                    if(autoStep == 1) { //Drive straight
                        if(gyro.getAngle() > -5 && gyro.getAngle() < 5) {
                            if (rightPosition <= -280*DRIVE_TO_INCHES && leftPosition <= -280*DRIVE_TO_INCHES ) {
                                leftMaster.set(ControlMode.PercentOutput, 0);
                                rightMaster.set(ControlMode.PercentOutput, .0);
                                if(timeStart) {
                                    autoTimer.reset();
                                    timeStart = false;
                                }
                                if(autoTimer.get() > .5) {
                                    autoStep++;
                                    zeroEncoders();
                                }
                            } else {
                                leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                                rightMaster.set(ControlMode.PercentOutput, -.75);
                            }
                    } else {
                        gyroAdjust(0, 5, .65, false);
                    }
                }
                    if(autoStep == 2) { //raise elevator
                        if(elevatorPosition < -32050 ) {
                            autoStep++;
                        } else {
                            elevator.set(ControlMode.Position, -33050);
                        }
                    }
                    if(autoStep == 3) { //rotate 50 degrees toward scale
                        gyroAdjust(-30, 5, .75, true);
                    }
                    if(autoStep == 4) { //spit out cube
                        if(!limitGrabber.get()) {
                            autoStep++;
                            grabRollerState(0, true);
                        } else {
                            grabRollerL.set(.4);
                            grabRollerR.set(.4);
                        }
                    }
                    if(autoStep == 5) {
                        if (rightPosition >= 12*DRIVE_TO_INCHES && leftPosition >= 12*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, -.50); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, .50);
                        }
                    }
                    if(autoStep == 6) {
                        elevator.set(ControlMode.Position, 0);
                        autoStep++;
                    }
                } else if(switchSide.equals("right")) {
                    if(autoStep == 1) { //Drive straight
                        if (rightPosition <= -145*DRIVE_TO_INCHES && leftPosition <= -145*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.75);
                        }
                    }
                    if(autoStep == 2) { //raise elevator
                        if(elevatorPosition < -11050 ) {
                            autoStep++;
                        } else {
                            elevator.set(ControlMode.Position, -12050);
                        }
                    }
                    if(autoStep == 3) { //rotate 90 degrees toward scale
                        gyroAdjust(-80, 5, .75, true);
                    }
                    if(autoStep == 4) {
                        if (rightPosition <= -20*DRIVE_TO_INCHES && leftPosition <= -20*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                grabberRotatePN.set(true);
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .8); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.8);
                        }
                    }
                    if(autoStep == 5) { //spit out cube
                        grabRollerState(1, false);
                        autoStep++;
                    }
                }else {
                    if(autoStep == 1) { //Drive straight
                        if(gyro.getAngle() > -5 && gyro.getAngle() < 5) {
                            if (rightPosition <= -180*DRIVE_TO_INCHES && leftPosition <= -180*DRIVE_TO_INCHES ) {
                                leftMaster.set(ControlMode.PercentOutput, 0);
                                rightMaster.set(ControlMode.PercentOutput, .0);
                                if(timeStart) {
                                    autoTimer.reset();
                                    timeStart = false;
                                }
                                if(autoTimer.get() > .5) {
                                    autoStep++;
                                }
                            } else {
                                leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                                rightMaster.set(ControlMode.PercentOutput, -.75);
                            }
                        } else {
                            gyroAdjust(0, 5, .75, false);
                        }
                    }
                    if(autoStep == 2) { //Nothing
                        autoStep++;
                    }
                    if(autoStep == 3) { //rotate 90 degrees toward scale
                        gyroAdjust(-90, 1, .75, true);
                    }
                    if(autoStep == 4) { //Nothing
                        autoStep++;
                    }
                }
                break;
       
            case 4: // TODO: Starting from the Left side going straight, then dropping the power cube into the scale. Possibly grabbing another cube.
                if (autoStep == 0) {
                    rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    zeroEncoders();
                    gyro.reset();
                    autoStep++;
                }
                if (scaleSide.equals("left")) {
                    if (autoStep == 1) {
                        if(gyro.getAngle() > -5 && gyro.getAngle() < 5) {
                        if (rightPosition <= -274*DRIVE_TO_INCHES && leftPosition <= -274*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .85); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.85);
                        }   
                        }
                        else {
                            gyroAdjust(0, 5, .75, false);
                        }
                    }
                    if (autoStep == 2) {
                        if(elevatorPosition < -31550 ) {
                            autoStep++;
                        } else {
                            elevator.set(ControlMode.Position, -33050);
                        }
                    }
                    if(autoStep == 3) {
                        gyroAdjust(25, 5, .75, true);
                        timeStart = true;
                    }
                   
                    if(autoStep == 5) {
                        if(timeStart) {
                            autoTimer.reset();
                            timeStart = false;
                        }
                        if(autoTimer.get() > .5) {
                            autoStep++;
                        }
                    }
                    if(autoStep == 4) {
                        grabRollerState(1, false);
                        autoStep++;
                    }
                    if(autoStep == 8) {
                        elevator.set(ControlMode.Position, -250);
                        autoStep++;
                    }
                } else { //right side
                    if (autoStep == 1) {
                        if (rightPosition <= -213*DRIVE_TO_INCHES && leftPosition <= -213*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .73); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.75);
                        }
                    }
                    if(autoStep == 2) {
                        gyroAdjust(0, 5, .75, true);
                    }
                    if (autoStep == 3) {
                        if (lateralPosition <= -226*LATERAL_TO_INCHES) {
                            zeroEncoders();
                            autoStep++;
                            lateralDeploy(false);
                        } else {
                            lateralDeploy(true);
                            lateralDrive.set(ControlMode.Position, .75);
                        }
                        //autoLateralDrive(226, LATERAL_TO_INCHES, .75);
                    }
                    if(autoStep == 4) {
                        if (rightPosition <= (-59 * DRIVE_TO_INCHES) && leftPosition <= (-59 * DRIVE_TO_INCHES)) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, 0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > 6) {
                                autoStep++;
                            }
                        } else {
                            rightMaster.set(ControlMode.PercentOutput, -.75);
                            leftMaster.set(ControlMode.PercentOutput, .75);
                        }
                    }
                    if(autoStep ==  5) {
                        if(elevatorPosition <= -33000) {
                            autoStep++;
                        } else {
                            elevator.set(ControlMode.Position, -34000 );
                        }
                    }
                    if(autoStep == 6) {
                        gyroAdjust(-40, 10, .75, true);
                    }
                   
                    if(autoStep == 7) {
                        grabRollerState(1, false);
                        autoStep++;
                    }
                    if(autoStep == 8) {
                        elevator.set(ControlMode.Position, -250);
                        autoStep++;
                    }
                }
                break;
            case 5: // TODO: Starting from the right side going straight, then dropping the power cube into the scale. Possibly grabbing another cube.
                if (autoStep == 0) {
                    zeroEncoders();
                    gyro.reset();
                    autoStep++;
                }
                if (autoStep == 1) {
                    if (rightPosition <= -28375 && leftPosition <= -22300 ) {
                        leftMaster.set(ControlMode.PercentOutput, 0);
                        rightMaster.set(ControlMode.PercentOutput, .0);
                        if(timeStart) {
                        autoTimer.reset();
                        timeStart = false;
                        }
                        if(autoTimer.get() > 2) {
                            autoStep++;
                        }
                    } else {
                        leftMaster.set(ControlMode.Position, 44366); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.Position, -51670);
                    }
                }
                if (!scaleSide.equals("left")) {
                    if (autoStep == 2) {
                        if (lateralPosition >= 1800) {
                            lateralDeploy(false);
                            lateralDrive.set(ControlMode.PercentOutput, 0);
                            zeroEncoders();
                            autoStep++;
                        } else {
                            lateralDeploy(true);
                            lateralDrive.set(ControlMode.Position, 1900);
                        }
                    }
                } else {
                    if (autoStep == 2) {
                        if (lateralPosition >= 14000) {
                            zeroEncoders();
                            autoStep++;
                            lateralDeploy(false);
                        } else {
                            lateralDeploy(true);
                            lateralDrive.set(ControlMode.Position, 15000);
                        }
                    }
                }
                if (autoStep == 3) {
                    if (elevatorPosition <= -24048) {
                        elevator.set(ControlMode.PercentOutput, 0); // 175.9 clicks per inch
                        zeroEncoders();
                        autoStep++;
                    } else {
                        elevator.set(ControlMode.Position, -25000); // 175.9 clicks per inch
                    }
                }
                if(autoStep == 4) {
                    if (leftPosition <= -6000 && rightPosition <= -5442) {
                        leftMaster.set(ControlMode.PercentOutput, 0); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.PercentOutput, 0);
                        zeroEncoders();
                        autoStep++;
                    } else {
                        leftMaster.set(ControlMode.Position, 20000); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.Position, -19000);
                    }
                }
                if (autoStep == 5) {
                /*    if (!limitGrabber.get()) {
                        autoStep++;
                        grabRollerState(0, false);
                    } else {
                        grabRollerState(1, false);
   
                    }*/
                grabRollerState(1, true);
                autoStep++;
                }
                if(autoStep == 6) {
                    if (leftPosition > 5000 && rightPosition > 5000) {
                        leftMaster.set(ControlMode.PercentOutput, 0); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.PercentOutput, 0);
                        zeroEncoders();
                        timeStart = true;
                        autoStep++;
                    } else {
                        leftMaster.set(ControlMode.Position, -20000); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.Position, 20000);
                    }
                }
                if (autoStep == 7) {
                    if(elevatorPosition > -100) {
                        autoStep++;
                    } else {
                        elevator.set(ControlMode.Position, 0);
                    }
                }
                if (autoStep == 8) {
                    if(gyro.getAngle() < 190 && gyro.getAngle() > 170) {
                        leftMaster.set(ControlMode.PercentOutput,0 ); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.PercentOutput,0);
                        autoStep++;
                    }else {
                        leftMaster.set(ControlMode.PercentOutput, -.75);
                        rightMaster.set(ControlMode.PercentOutput, -.75);       
                    }
                }
                if( autoStep == 9) {
                    if (leftPosition > -1500 && rightPosition > 1500) {
                        leftMaster.set(ControlMode.PercentOutput, 0); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.PercentOutput, 0);
                        zeroEncoders();
                        autoStep++;
                    }else {
                        leftMaster.set(ControlMode.Position, -1600); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.Position, -1600);
                        grabberRotatePN.set(true);
                    }
                }       
                if( autoStep == 10) {
                    grabRollerState(-1, true);
                    autoTimer.reset();
                    if(autoTimer.get() > .1) {
                        grabRollerState(0, false);
                        autoStep++;
                    }
                }
                if(autoStep == 11) {
                    if(elevatorPosition < -10304 ) {
                        autoStep++;   
                    }else {
                        elevator.set(ControlMode.Position, -11000);
                    }
                }
                if(autoStep == 12) {
                    autoTimer.reset();
                    if(autoTimer.get() > .05) {
                        leftMaster.set(ControlMode.PercentOutput, 0);
                        rightMaster.set(ControlMode.PercentOutput, 0);
                        if(!limitGrabber.get()) {
                            grabRollerState(0, false);
                            autoStep++;
                        } else {
                            grabRollerState(1, false);
                        }
                    } else {
                        leftMaster.set(ControlMode.PercentOutput, .75);
                        rightMaster.set(ControlMode.PercentOutput, .75);
                    }
                }
                if(autoStep == 13) {
                    if(rightPosition < -1500 && leftPosition > 1500) {
                        autoStep++;
                        zeroEncoders();
                    } else {
                        rightMaster.set(ControlMode.Position, -1600);
                        leftMaster.set(ControlMode.Position, 1600);
                        grabberRotatePN.set(false);
                    }
                }
                break;
            case 6: //TODO: straight and load the switch based on the FMS
                if(autoStep == 0) {
                    zeroEncoders();
                    gyro.reset();
                    autoStep++;
                    timeStart = true;
                }
                if(autoStep == 1) {
                    if(rightPosition < (-36*DRIVE_TO_INCHES) && leftPosition < (-36*DRIVE_TO_INCHES)) {
                        autoStep++;
                        zeroEncoders();
                        rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
                        leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
                    } else {
                        rightMaster.set(ControlMode.PercentOutput, -.75);
                        leftMaster.set(ControlMode.PercentOutput, .75);
                    }
                    /*
                     *
                     */
                }
                if(switchSide.equals("left")) {
                    if (autoStep == 2) {
                        if (lateralPosition >= 3000) {
                            lateralDeploy(false);
                            lateralDrive.set(ControlMode.PercentOutput, 0);
                            zeroEncoders();
                            autoStep++;
                        } else {
                            lateralDeploy(true);
                            lateralDrive.set(ControlMode.Position, -1900);
                        }
                    }
                } else {
                    if (autoStep == 2) {
                        if (lateralPosition <= -2600) {
                            zeroEncoders();
                            autoStep++;
                            lateralDeploy(false);
                           
                        } else {
                            lateralDeploy(true);
                            lateralDrive.set(ControlMode.Position, 1900);
                        }
                    }
                }
                if(autoStep == 3) {
                    if(elevatorPosition <=  -7000) {
                        autoStep++;
                    } else {
                        elevator.set(ControlMode.Position, -10000 );
                    }
                }
                if(autoStep == 4) {
                    if(rightPosition < (-48.5*DRIVE_TO_INCHES) && leftPosition < (-48.5*DRIVE_TO_INCHES)) {
                        autoStep++;
                        zeroEncoders();
                        time = autoTimer.get();
                    } else {
                        rightMaster.set(ControlMode.PercentOutput, -1.0);
                        leftMaster.set(ControlMode.PercentOutput, 1.0);
                    }
                }
                if(autoStep == 5) {
                    if(autoTimer.get() >= (time + .5)) {
                        autoStep++;
                    }
                }
                if(autoStep == 5) {
                grabRollerState(1, false);
                autoStep++;
                }
                break;
            case 3: //TODO: to scale or switch based on FMS double cube on scale
                if (autoStep == 0) {
                    rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    zeroEncoders();
                    gyro.reset();
                    autoStep++;
                }
                if (scaleSide.equals("left")) {
                    if (autoStep == 1) { //Drive straight
                        if (gyro.getAngle() > -5 && gyro.getAngle()< 5) {
                            if (rightPosition <= -250*DRIVE_TO_INCHES && leftPosition <= -250*DRIVE_TO_INCHES ) {
                                leftMaster.set(ControlMode.PercentOutput, 0);
                                rightMaster.set(ControlMode.PercentOutput, .0);
                                if(timeStart) {
                                    autoTimer.reset();
                                    timeStart = false;
                                }
                                if(autoTimer.get() > .5) {
                                    autoStep++;
                                    zeroEncoders();
                                }
                            } else {
                                leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                                rightMaster.set(ControlMode.PercentOutput, -.75);
                            }
                        } else {
                            gyroAdjust(0, 5, .75, false);
                        }
                    }
                    if(autoStep == 2) { // raise elevator
                        if(elevatorPosition < -32050 ) {
                            autoStep++;
                        } else {
                            elevator.set(ControlMode.Position, -33050);
                        }
                    }
                    if(autoStep == 3) { //lateral 10 in
                        if(lateralPosition > 10*LATERAL_TO_INCHES) {
                            lateralDrive.set(ControlMode.PercentOutput, 0);
                            zeroEncoders();
                            autoStep++;
                        } else {
                            lateralDeploy(true);
                            lateralDrive.set(ControlMode.PercentOutput, -.75);
                        }
                    }
                    if(autoStep == 4) { // un-succ cube
                        if(!limitGrabber.get()) {
                            autoStep++;
                            grabRollerState(0, true);
                        } else {
                            grabRollerL.set(.4);
                            grabRollerR.set(.4);
                        }
                    }
                    if(autoStep == 5) { //go 12 in backwards
                        if (rightPosition >= 41.5*DRIVE_TO_INCHES && leftPosition >= 41.5*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, -.50); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, .50);
                        }
                    }
                    if(autoStep == 6) { //rotate to 180
                        gyroAdjust(180, 5, .75, true);
                    }
                    if(autoStep == 7) { // move elevator to pick-up position
                        if(elevatorPosition >= -300) {
                            autoStep++;
                            grabberRotatePN.set(true);
                        } else {
                            elevator.set(ControlMode.Position, -225);
                        }
                    }
                    if(autoStep == 8) { //going forward 12 in
                        if (rightPosition <= -41.5*DRIVE_TO_INCHES && leftPosition <= -41.5*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .50); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.50);
                        }
                    }
                    if(autoStep == 9) { // succ in the cube
                        if(limitGrabber.get()) {
                            autoStep++;
                            grabRollerState(0, true);
                        } else {
                            grabRollerState(-1, true);
                        }
                    }
                    if(autoStep == 10) { //adjust to 0 degrees
                        gyroAdjust(0, 5, .75, true);
                        elevator.set(ControlMode.Position, -33050);
                    }
                    if(autoStep == 11) { //Drive forward 24 in
                        if (rightPosition <= -83*DRIVE_TO_INCHES && leftPosition <= -83*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .50); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.50);
                        }
                    }
                    if(autoStep == 12) { // un-succ the cube
                        if(!limitGrabber.get()) {
                            autoStep++;
                            grabRollerState(0, true);
                        } else {
                            grabRollerL.set(.4);
                            grabRollerR.set(.4);
                        }
                    }
                } else if(switchSide.equals("left")) {
                    if(autoStep == 1) { //Drive straight
                        if (rightPosition <= -155*DRIVE_TO_INCHES && leftPosition <= -155*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.75);
                        }
                    }
                    if(autoStep == 2) { //raise elevator
                        if(elevatorPosition < -11050 ) {
                            autoStep++;
                        } else {
                            elevator.set(ControlMode.Position, -12050);
                        }
                    }
                    if(autoStep == 3) { //rotate 80 degrees toward scale
                        gyroAdjust(80, 5, .75, true);
                    }
                    if(autoStep == 4) { // drive 20 in and deploy grabber
                        if (rightPosition <= -20*DRIVE_TO_INCHES && leftPosition <= -20*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                grabberRotatePN.set(true);
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.75);
                        }
                    }
                    if(autoStep == 5) { //spit out cube
                        grabRollerState(1, false);
                        autoStep++;
                    }
                }else {
                    if(autoStep == 1) { //Drive straight
                        if(gyro.getAngle() > -5 && gyro.getAngle() < 5) {
                            if (rightPosition <= -180*DRIVE_TO_INCHES && leftPosition <= -180*DRIVE_TO_INCHES ) {
                                leftMaster.set(ControlMode.PercentOutput, 0);
                                rightMaster.set(ControlMode.PercentOutput, .0);
                                if(timeStart) {
                                    autoTimer.reset();
                                    timeStart = false;
                                }
                                if(autoTimer.get() > .5) {
                                    autoStep++;
                                }
                            } else {
                                leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                                rightMaster.set(ControlMode.PercentOutput, -.75);
                            }
                        } else {
                            gyroAdjust(0, 5, .75, false);
                        }
                    }
                    if(autoStep == 2) { //raise elevator
                        autoStep++;
                    }
                    if(autoStep == 3) { //rotate 90 degrees toward scale
                        gyroAdjust(90, 1, .75, true);
                    }
                    if(autoStep == 4) { //spit out cube
                        autoStep++;
                    }
                }
                break;
            case 7: //TODO: cross scale
                if (autoStep == 0) {
                    rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
                    zeroEncoders();
                    gyro.reset();
                    autoStep++;
                }
                if(autoStep == 1) { //Drive straight
                    if(gyro.getAngle() > -5 && gyro.getAngle() < 5) {
                        if (rightPosition <= -208*DRIVE_TO_INCHES && leftPosition <= -208*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                autoStep++;
                                zeroEncoders();
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.75);
                        }
                    } else {
                        gyroAdjust(0, 5, .65, false);
                    }
                }
                if(scaleSide.equals("left")) {
                    if(autoStep == 2) {
                        if(lateralPosition <= -60*LATERAL_TO_INCHES) { //lateral to left side
                            lateralDeploy(false);
                            lateralDrive.set(ControlMode.PercentOutput, 0);
                            zeroEncoders();
                            autoStep++;
                        } else {
                            lateralDeploy(true);
                            lateralDrive.set(ControlMode.PercentOutput, .6);
                            elevator.set(ControlMode.Position, -33050);
                        }
                    }
                } else {
                    if(autoStep == 2) {
                        if(lateralPosition <= -155*LATERAL_TO_INCHES) { //lateral to right side
                            lateralDeploy(false);
                            lateralDrive.set(ControlMode.PercentOutput, 0);
                            zeroEncoders();
                            autoStep++;
                        } else {
                            lateralDeploy(true);
                            lateralDrive.set(ControlMode.PercentOutput, 1);
                            elevator.set(ControlMode.Position, -33050);
                        }
                    }
                }
                if(autoStep == 3) { // drive 20 in to scale
                    if (rightPosition <= -40*DRIVE_TO_INCHES && leftPosition <= -40*DRIVE_TO_INCHES ) {
                        leftMaster.set(ControlMode.PercentOutput, 0);
                        rightMaster.set(ControlMode.PercentOutput, .0);
                        if(timeStart) {
                            autoTimer.reset();
                            timeStart = false;
                        }
                        if(autoTimer.get() > .5) {
                            autoStep++;
                            zeroEncoders();
                        }
                    } else {
                        leftMaster.set(ControlMode.PercentOutput, .75); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.PercentOutput, -.75);
                    }
                }
                if(autoStep == 4) { //spit out cube
                    if(!limitGrabber.get()) {
                        autoStep++;
                        grabRollerState(0, true);
                    } else {
                        grabRollerL.set(.4);
                        grabRollerR.set(.4);
                    }
                }
                if(autoStep == 5) {
                    if (rightPosition >= 12*DRIVE_TO_INCHES && leftPosition >= 12*DRIVE_TO_INCHES ) {
                        leftMaster.set(ControlMode.PercentOutput, 0);
                        rightMaster.set(ControlMode.PercentOutput, .0);
                        if(timeStart) {
                            autoTimer.reset();
                            timeStart = false;
                        }
                        if(autoTimer.get() > .5) {
                            zeroEncoders();
                            autoStep++;
                        }
                    } else {
                        leftMaster.set(ControlMode.PercentOutput, -.50); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.PercentOutput, .50);
                    }
                }
                if(autoStep == 6) { //rotate to 180
                    gyroAdjust(180, 5, .75, true);
                }
                if(autoStep == 7) {
                    if(elevatorPosition >= -300) {
                        autoStep++;
                        grabberRotatePN.set(true);
                    } else {
                        elevator.set(ControlMode.Position, -225);
                    }
                }
                if(autoStep == 8) { //going forward 12 in
                    if (rightPosition <= -41.5*DRIVE_TO_INCHES && leftPosition <= -41.5*DRIVE_TO_INCHES ) {
                        leftMaster.set(ControlMode.PercentOutput, 0);
                        rightMaster.set(ControlMode.PercentOutput, .0);
                        if(timeStart) {
                            autoTimer.reset();
                            timeStart = false;
                        }
                        if(autoTimer.get() > .5) {
                            zeroEncoders();
                            autoStep++;
                        }
                    } else {
                        leftMaster.set(ControlMode.PercentOutput, .50); // 175.9 clicks per inch
                        rightMaster.set(ControlMode.PercentOutput, -.50);
                    }
                }
                if(autoStep == 9) { // succ in the cube
                    if(limitGrabber.get()) {
                        autoStep++;
                        grabRollerState(0, true);
                    } else {
                        grabRollerState(-1, true);
                    }
                }
                if(switchSide.equals(scaleSide)) { //if switch and scale are on the same side
                    if(autoStep == 10) { //raise elevator
                        elevator.set(ControlMode.Position, -12000);
                        if(elevatorPosition <= -10000) {
                            autoStep++;
                        }
                    }
                    if(autoStep == 11) {
                        grabRollerState(1, false);
                    }
                } else {
                    if(autoStep == 10) {
                        if (rightPosition <= 3*DRIVE_TO_INCHES && leftPosition <= 3*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, -.50); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, .50);
                        }
                    }
                    if(autoStep == 11) { //adjust to 0 degrees
                        gyroAdjust(0, 5, .75, true);
                    }
                    if(autoStep == 12) { //Drive forward 24 in
                        if (rightPosition <= -83*DRIVE_TO_INCHES && leftPosition <= -83*DRIVE_TO_INCHES ) {
                            leftMaster.set(ControlMode.PercentOutput, 0);
                            rightMaster.set(ControlMode.PercentOutput, .0);
                            if(timeStart) {
                                autoTimer.reset();
                                timeStart = false;
                            }
                            if(autoTimer.get() > .5) {
                                zeroEncoders();
                                autoStep++;
                            }
                        } else {
                            leftMaster.set(ControlMode.PercentOutput, .70); // 175.9 clicks per inch
                            rightMaster.set(ControlMode.PercentOutput, -.70);
                            elevator.set(ControlMode.Position, -33050);
                        }
                    }
                    if(autoStep == 13) { // un-succ the cube
                        if(!limitGrabber.get()) {
                            autoStep++;
                            grabRollerState(0, true);
                        } else {
                            grabRollerL.set(.4);
                            grabRollerR.set(.4);
                        }
                    }
                }
                break;
                /*
                if(autoStep == 6) {
                     lateralDeploy(true);
                     lateralDrive.set(ControlMode.Position, 3000);
                      if(!irManipLeft.get() && !irManipRight.get()) {
                           zeroEncoders(); autoStep++; lateralDeploy(false);
                    }
                  }
                 */
        }

        //System.out.println("Right Output: " + rightMaster.getMotorOutputPercent());
        //System.out.println("Left Output: " + leftMaster.getMotorOutputPercent());

        System.out.println("Right Side: " + rightPosition);
        System.out.println("Left Side: " + leftPosition);
        System.out.println("Lateral: " + lateralPosition);
        System.out.println("Auto Step: " + autoStep);
        System.out.println("Gyro: " + gyro.getAngle());
        System.out.println("AutoType: " + autoType);
        System.out.println("Elevator Height: " + elevatorPosition);
       
        SmartDashboard.putNumber("Auto Step", autoStep);
        SmartDashboard.putNumber("Elevator Height", elevatorPosition);
        SmartDashboard.putNumber("Right Side Position", rightPosition);
        SmartDashboard.putNumber("Left Side Position", leftPosition);
       
        //================================================================================
        // This is how Jamie codes!
        //================================================================================
        /*char[] hello = new char[5];
        hello[0] = 'h';
        hello[1] = 'e';
        hello[2] = 'l';
        hello[3] = 'l';
        hello[4] = 'o';
        int len;
        len = hello.length;
        int x;
        x = 0;
        while (x < len) {
            System.out.print(hello[x]);
            x++;
        }*/

    }

    /**
     * This function is called once each time the robot enters teleoperated mode.
     */
    @Override
    public void teleopInit() {
        rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        grabberRotatePN.set(true);
        climbAlign.set(false);
       
    }
   

    /**
     * This function is called periodically during teleoperated mode.
     */
    @Override
    public void teleopPeriodic() {
       
        //System.out.println(limitGrabber.get());

        //================================================================================
        // Drive
        //================================================================================
        if (rightStick.getTrigger()) { // TODO: When right trigger is pressed, lateral drive is deployed and is controlled by Joystick
            lateralDeploy(true);
          /*    if(elevator.getSelectedSensorPosition(ELEVATOR_POSITION) > -25555) {
                 lateralDrive.set(ControlMode.PercentOutput, rightStick.getY() * (-25555/elevator.getSelectedSensorPosition(ELEVATOR_POSITION)));
             } else {
                 lateralDrive.set(ControlMode.PercentOutput, rightStick.getY());
             }*/
            lateralDrive.set(ControlMode.PercentOutput, rightStick.getX()); //forward = left backward = right
        } else { // TODO: When trigger is not pressed regular drive is set, and is controlled by the joystick
            lateralDeploy(false);
         /*    if(elevator.getSelectedSensorPosition(ELEVATOR_POSITION) > -25555) {
                 RobotDrive.arcadeDrive((rightStick.getY() *(-25555/elevator.getSelectedSensorPosition(ELEVATOR_POSITION))), rightStick.getX());
             } else {
                 RobotDrive.arcadeDrive(rightStick.getY(), rightStick.getX());
            }*/
            RobotDrive.arcadeDrive(rightStick.getY(), rightStick.getX());
        }
       
       
        rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);

        System.out.println("Right Side: " + rightMaster.getSelectedSensorPosition(RIGHT_DRIVE_POSITION));
        System.out.println("Left Side: " + leftMaster.getSelectedSensorPosition(LEFT_DRIVE_POSITION));
        System.out.println("Lateral: " + lateralDrive.getSelectedSensorPosition(LATERAL_DRIVE_POSITION));
        System.out.println("Lateral Power: " + lateralDrive.getMotorOutputPercent());

        if (rightStick.getRawButton(2) && shiftPress) {
            shiftToggle = shiftToggle ? false : true;
            shiftPress = false;
        }
       
         shift.set(rightStick.getRawButton(2)); // true = high speed & false = low speed
                                                 // TODO: When driver button 2 is pressed the robot shifts gears
        //================================================================================
        // Power Cube Manipulator
        //================================================================================
        if (leftStick.getTrigger()) { // TODO:when operators trigger is pressed rollers suck in
            grabRollerState(-1, true);
            grabOut = false;
        } else if (leftStick.getRawButton(3)) { // TODO: when operators trigger is pressed rollers push out
            grabRollerState(1, true);
            grabOut = true;
        } else if (leftStick.getRawButton(4)) {
            grabRollerState(1, false);
            grabOut = true;
        } else { // Grabber bias
              //grabRollerState(0, true); // 0.2
            grabRollerL.set(-.15);
            grabRollerR.set(-.15);
            grabOut = false;
        }
        if (leftStick.getRawButton(5) && grabRotatePress) { // TODO: toggle grabber rotate. press once gets you to 90, press again brings you back to 0.
            //grabRotateToggle = (grabRotateToggle ? false : true);
                grabRotateToggle = grabOut ? grabRotateToggle : !grabRotateToggle;
            //grabRotateToggle = !grabRotateToggle;
            grabRotatePress = false;
        }
        if (!leftStick.getRawButton(5)) {
            grabRotatePress = true;
        }
        if(alignOut) {
            grabRotateToggle = false;
        }
        if (grabRotateToggle) {
                grabberRotatePN.set(true);
        } else {
            grabberRotatePN.set(false); //true = down
        }
        
        //================================================================================
        // Climb
        //================================================================================
        if (rightStick.getRawButton(7) && climbDown.get()) { //TODO: when the driver holds button 7, extends until limit is hit
            climbExtendSR.set(1); //climber extender is set to 100%
        } else if (rightStick.getRawButton(8) && climbUp.get()) { //TODO: when the driver holds button 8, retracts until limit is hit
            climbExtendSR.set(-1); //climber extender is set to -100%
        } else {
            climbExtendSR.set(0); //climber extender is set to zero
        }
       
        if (climbUp.get()) {
            System.out.print("climbUp true");
        } else {
            //System.out.print("climbUp false");
        }
        if (climbDown.get()) {
            System.out.println("climbDown true");
        } else {
            //System.out.println("climbDown false");
        }
   
            //winch.set(rightStick.getRawButton(11) ? (-rightStick.getThrottle() + 1) * 0.5 : rightStick.getRawButton(3) && rightStick.getRawButton(4) ? -1 : 0); // TODO: when operate button 11 is pressed it sets the winch to start winch-ing
        winch.set(rightStick.getRawButton(11) ? 1 : rightStick.getRawButton(3) && rightStick.getRawButton(4) ? -1 : 0); // TODO: when operate button 11 is pressed it sets the winch to start winch-ing
        if (rightStick.getRawButton(9) && alignPress) { // TODO: when operate button 9 is pressed the climber alignment is toggled
            alignToggle = (alignToggle ? false : true);
            alignPress = false;
        }
        if (!rightStick.getRawButton(9)) {
            alignPress = true;
        }
        if(winch.get() != 0) {
            alignToggle = false;
        }
        if (alignToggle) {
            climbAlign.set(true);
            elevator.set(ControlMode.Position, 0); //elevator is completely lowered
        } else {
            climbAlign.set(false); //climber is pulled in
        }
        if(climbAlign.get()) {
            alignOut = true;
        } else {
            alignOut = false;
        }

        //================================================================================
        // Elevator
        //================================================================================
        if (leftStick.getRawButton(12)) {// TODO: when the operator holds button 4 elevator is controlled on a Percent Output based on joystick
                if (leftStick.getY() <= 0) {
                    elevator.set(ControlMode.PercentOutput, leftStick.getY() * 1);
                } else {
                    elevator.set(ControlMode.PercentOutput,  leftStick.getY() * 0.6);
                   
                }
            wantedPositionRotations = elevator.getSelectedSensorPosition(ELEVATOR_POSITION);
        } else {
            wantedPositionRotations
                    = leftStick.getRawButton(2) ? -225 //Just above the ground
                    : leftStick.getRawButton(7) ? -1926 // exchange position -900
                    : leftStick.getRawButton(8) ? -12000 // switch -9938 TODO: when the operator presses button 7 elevator set to go 25%
                    : leftStick.getRawButton(9) ? -25555 // Low scale       TODO: when the operator presses button 8 elevator set to go 50%
                    : leftStick.getRawButton(10) ? -33050 //  High Scale    TODO: when the operator presses button 9 elevator set to go 75%
                    : wantedPositionRotations;
            elevator.set(ControlMode.Position, wantedPositionRotations); // sets position of the elevator to the value to targetPositionRotations   
        }
        if(!limitGrabber.get()) {
            canElevatorBeRaised = true;
        }
        if(limitGrabber.get() && canElevatorBeRaised) {
            canElevatorBeRaised = false;
            elevator.set(ControlMode.Position, -600);
        }
        elevator.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        elevator.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, 10);
        elevator.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 10);
        System.out.println("Elevator Position: " + elevator.getSelectedSensorPosition(ELEVATOR_POSITION));
        System.out.println("Elevator Current: " + elevator.getOutputCurrent());
        System.out.println("Elevator Voltage: " + elevator.getMotorOutputVoltage());
        SmartDashboard.putNumber("Elevator Voltage:", elevator.getMotorOutputVoltage() );
        SmartDashboard.putNumber("Elevator Current:", elevator.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Encoder: ", elevator.getSelectedSensorPosition(ELEVATOR_POSITION));
        //================================================================================
        // Lights/IR
        //================================================================================
        grabberLight.set(limitGrabber.get() ? Relay.Value.kForward : Relay.Value.kOff); // while IR sees something, the lights are turned on
        relayThree.set(irManipRight.get() && irManipLeft.get() && deploy ? Relay.Value.kForward : Relay.Value.kOff);
        if (!irManipRight.get() && !irManipLeft.get() && deploy) {
            System.out.println("exchange is lined up");
        }
        if (irManipRight.get()) {
            System.out.println("right true");
           
        } else {
            //System.out.println("right false");
        }
        if (irManipLeft.get()) {
            System.out.println("left true");
        } else {
            //System.out.println("left false");
        }
        SmartDashboard.putBoolean("Do we have a cube?", limitGrabber.get());

        //================================================================================
        // Zero Sensors
        //================================================================================
        if (rightStick.getRawButton(3)) { //TODO:
            zeroEncoders();
            //gyro.reset();
            elevator.setSelectedSensorPosition(0, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
        }
       
        //================================================================================
        // Auto Switches
        //================================================================================
        if (rightStick.getPOV() == 0) { //TODO:
            autoRightServo.set(.91);
            autoLeftServo.set(.35);
        } else if (rightStick.getPOV() == 90) {
            autoRightServo.set(.409);
            autoLeftServo.set(.84);
        }
        System.out.println("Gyro: " + gyro.getAngle());
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        //RobotDrive.tankDrive( , zRotation);
        //================================================================================
        // Motorcycle Mode
        //================================================================================
        RobotDrive.tankDrive(leftStick.getY(), rightStick.getY());
       
        if(leftStick.getTrigger()) {
            grabRollerL.set(leftStick.getZ());
            grabRollerR.set(leftStick.getZ());
        if(leftStick.getTrigger()) {
           
        }
        }
        
       
    }

    /**
     * Moves robot autonomously based on desired encoder counts.
     * @param leftPos is the amount of encoder counts to turn on left side
     * @param rightPos is the amount of encoder counts to turn on right side
     * @param stopPos is the desired end-point
     * @param isStraight determines whether to stop when right and left sides at stopPos or only one side
     */
    public void autoDrive(int leftPos, int rightPos, int stopPos, boolean isStraight, RobotType robotType) {
        // Drive robot based on number of left and right counts
        leftMaster.set(ControlMode.Position, leftPos);
        rightMaster.set(ControlMode.Position, rightPos);
       
        // Move onto next auto step based on leftPosition if turnDirection = true or rightPosition if turnDirection = false
        if (isStraight) { // Not Jamie!
            if (rightPosition >= stopPos && leftPosition <= -stopPos) {
                autoStep++;
                zeroEncoders();
            }
        } else {
            if (Math.abs(rightPos) < Math.abs(leftPos)) { //this would make the code more intuitive instead of having to input that extra boolean
                if (leftPosition <= stopPos) {
                    autoStep++;
                    zeroEncoders();
                }
            } else {
                if (rightPosition >= stopPos) {
                    autoStep++;
                    zeroEncoders();
                }
            }
        }
    }

    /**
     * Moves robot autonomously to given encoder count using lateral drive.
     * @param stopPos is the desired end-point
     * @param d is any value above stopPos
     */
    public void autoLateralDrive(double inches, double conversion, double speed) {
        // Stop lateral based on stopPos
        if (Math.abs(lateralPosition) >= Math.abs(inches * conversion)) {
            zeroEncoders();
            autoStep++;
            // This method stops lateral drive and deploys regular drive system
            lateralDeploy(false);
        } else {
            // Stop regular drive
            lateralDeploy(true);
            // Move lateral drive based on number of lateral counts
            lateralDrive.set(ControlMode.PercentOutput, speed);
        }
    }
   
    /**
     * Deploys Lateral Drive.
     */
    /*public void lateralDeploy() {
        RobotDrive.arcadeDrive(0, 0);
        lateralDeploy.set(true);
        deploy = true;
    }*/

    /**
     * Retracts Lateral Drive.
     */
    /*public void lateralUnDeploy() {
        lateralDrive.set(ControlMode.PercentOutput, 0);
        lateralDeploy.set(false);
        deploy = false;
    }*/
   
    /**
     * Deploys lateral drive or retracts based on input
     * @param deploy specifies whether to retract or deploy lateral drive
     */
    public void lateralDeploy(boolean deploy) {
        if (deploy) {
            RobotDrive.arcadeDrive(0, 0);
            lateralDeploy.set(true);
            deploy = true;
        } else {
            lateralDrive.set(ControlMode.PercentOutput, 0);
            lateralDeploy.set(false);
            deploy = false;
        }
    }

    /**
     * Resets encoder counts on drive motors.
     */
    public void zeroEncoders() {
        leftMaster.setSelectedSensorPosition(0, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
        rightMaster.setSelectedSensorPosition(0, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
        lateralDrive.setSelectedSensorPosition(0, K_PID_LOOP_INDEX, K_TIMEOUT_MS);
    }
   
    /**
     * Grabs or retracts power cube.
     * @param grabToggle determines whether to grab or retract power cube
     */
    public void grabRollerState(int state, boolean fast) {
        if(state < 0) {
            grabRollerL.set(-1);
            grabRollerR.set(-1);
        } else if(state > 0) {
            grabRollerL.set(fast ? 1 : .3);
            grabRollerR.set(fast ? 1 : .3);
        } else if(state == 0) {
            grabRollerL.set(fast && !limitGrabber.get() ? -.2 : 0);
            grabRollerR.set(fast && !limitGrabber.get() ? -.2 : 0);
        }
    }
   
    public void gyroAdjust(double goal, int deadband, double rate, boolean stepAfter) {
        if(gyro.getAngle() > goal) {
            if(gyro.getAngle() >= goal-deadband && gyro.getAngle() < goal+deadband) {
                leftMaster.set(ControlMode.PercentOutput, 0);
                rightMaster.set(ControlMode.PercentOutput, 0);
                if(stepAfter) {
                    autoStep++;
                }
                zeroEncoders();
            } else {
                leftMaster.set(ControlMode.PercentOutput, -rate);
                rightMaster.set(ControlMode.PercentOutput, -rate);
            }
        } else {
            if(gyro.getAngle() >= goal-deadband && gyro.getAngle() < goal+deadband) {
                leftMaster.set(ControlMode.PercentOutput, 0);
                rightMaster.set(ControlMode.PercentOutput, 0);
                if(stepAfter) {
                    autoStep++;
                }
                zeroEncoders();
            } else {
                leftMaster.set(ControlMode.PercentOutput, rate);
                rightMaster.set(ControlMode.PercentOutput, rate);
            }
        }
    }
   
    public void autoGrabRollerState(double state, boolean fast) {
        if(state < 0) { //input
            grabRollerL.set(!limitGrabber.get() ? -1 : 0);
            grabRollerR.set(!limitGrabber.get() ? -1 : 0);
        } else if(state > 0) { //output
            if(fast) {
                grabRollerL.set(!limitGrabber.get() ? 1 : 0);
                grabRollerR.set(!limitGrabber.get() ? 1 : 0);
            } else {
                grabRollerL.set(!limitGrabber.get() ? .5 : 0);
                grabRollerR.set(!limitGrabber.get() ? .5 : 0);
            }
        } else if(state == 0) { //static
            grabRollerL.set(0);
            grabRollerR.set(0);
        }
    }

}