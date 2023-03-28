package frc.robot.test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.test.arm.Arm;
import frc.robot.test.autonomous.Autonomous;
import frc.robot.test.drive.Drive;
import frc.robot.test.hardware.Hardware;
import frc.robot.test.vision.Vision;

public abstract class Robot {
	/**
	<Strong>FULL_SUBSYSTEM_TESTS: </Strong> All subsytems. Takes longer, but is comprehensive. <p>
	<Strong>DRIVE_TESTS: </Strong> Drive subsystem tested. Includes turning and driving. <p>
	<Strong>AUTONOMOUS_TESTS: </Strong> Autonomous subsystem tested. Includes going through all routine and requires user input. <p> 
	<Strong>VISION_TESTS: </Strong> Vision subsystem tested. Includes testing apriltag detection with user input. <p>
	<Strong>ARM_TESTS: </Strong> Arm subsystem tested. Include moving arm to each position with user input to confirm. <p>
	<Strong>HARWARD_TEST: <Strong> RoboRIO & Battery Voltage tested. Shortest to run and only includes essential electronics. 
	**/
	public enum TEST_TYPES {
		FULL_SUBSYSTEM_TESTS,
		DRIVE_TESTS,
		AUTONOMOUS_TESTS,
		VISION_TESTS,
		ARM_TESTS,
		HARDWARE_TESTS,
	}

	public static Command runSystemChecks(TEST_TYPES test_type){
		switch(test_type){
			case FULL_SUBSYSTEM_TESTS:
				return new SequentialCommandGroup(
					// Run All Checks
					runDriveChecks(),
					runAutonomousChecks(),
					runVisionChecks(),
					runArmChecks(),
					runHardwareChecks()
				);
			case DRIVE_TESTS:
				return runDriveChecks();
			case AUTONOMOUS_TESTS:
				return runAutonomousChecks();
			case VISION_TESTS:
				return runVisionChecks();
			case ARM_TESTS:
				return runArmChecks();
			case HARDWARE_TESTS:
				return runHardwareChecks();
		}
        System.out.println("[WARNING]: Subsystem error has occured. No checks have been run.");
        return null;
	}
	/**
	<Strong>Part One: </Strong> Drivetrain runs foward starting at speed 0.1. Every two second the speed of the drivetrain is 
	increased by 0.1 until the full speed of 1 is reached. <p>
	<Strong>Part Two: </Strong> Drivetrain run backward starting at speed -0.1. Every two second the speed of the drivetrain is 
	decreased by 0.1 until the full speed of -1 is reached. <p>
	<Strong>Part Three: </Strong> Drivetrain slowly makes a full revolution clockwise. Next, a full rotation is made 
	counterclockwise slowly. Finally the same previous checks occur yet speed is incresed. <p>
	<Strong>Part Four: </Strong> The last checks is a combination of all previous tests combined occuring at the same time to 
	simulate game play. <p>
	<Strong>Part Five: </Strong> Terminal outputs results from testing including preformance report. 
	**/
	private static Command runDriveChecks(){
        return Drive.runChecks();
	}
	
	/**
	<Strong>Part One: </Strong> All autonomous routines that are define will be run. <p>
	<Strong>Part Two: </Strong> Preformance report will be outputted to the terminal.
	**/
	private static Command runAutonomousChecks(){
        return Autonomous.runChecks();
	}
	
	/**
	<Strong>Part One: </Strong> 
	**/
	private static Command runVisionChecks(){
        return Vision.runChecks(null);
	}
	
	private static Command runArmChecks(){
        return Arm.runChecks(null);
	}
	
	private static Command runHardwareChecks(){
        return Hardware.check();
	}
}
