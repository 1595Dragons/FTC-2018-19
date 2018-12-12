package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Stephen Ogden on 12/11/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@Autonomous(name = "Auto Sample test", group = "Test")
public class Auto_SampleTest extends LinearOpMode {

    private static final double DRIVE_SPEED = .15, TURN_SPEED = 1, ARM_SPEED = .8, SIDE_SPEED = .25;

    // Config for the robot
    private Config robot = new Config(this);


    @Override
    public void runOpMode() {

        // Setup robot hardware
        robot.ConfigureRobtHardware(false);


        // Send telemetry message to signify robot waiting;
        robot.status("Resetting motors");
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front, robot.armMotorL, robot.armMotorR);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //PlanA
        robot.armDrive(ARM_SPEED, 680, 3);
        sleep(200);

        //go left 10
        robot.autoDriveSideways(SIDE_SPEED, -10, robot.getAngle(), 3);
        sleep(200);

        //forward 8
        robot.autoDriveStraight(DRIVE_SPEED, 8, robot.getAngle(), 3);
        sleep(200);

        //turn left 4
        robot.autoDriveSideways(DRIVE_SPEED, -4, robot.getAngle(), 3);
        sleep(200);

        //left 13
        robot.autoDriveSideways(SIDE_SPEED, -13, robot.getAngle(), 3);

        robot.setupGoldDetector();
        robot.goldDetector.enable();

        int count = 0;
        while (!robot.goldDetector.isFound() && count < 36) {
            robot.autoDriveSideways(SIDE_SPEED, 1, robot.getAngle(), 1);
            count++;
        }

        // RAMMING SPEED! lol jk
        robot.autoDriveStraight(DRIVE_SPEED, 36, robot.getAngle(), 3);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
