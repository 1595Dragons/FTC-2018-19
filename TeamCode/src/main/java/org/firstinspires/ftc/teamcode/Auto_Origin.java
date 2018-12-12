package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Origin", group = "Test")
public class Auto_Origin extends LinearOpMode {

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
        robot.distinctDrive(SIDE_SPEED, 10, -10, -10, 10, 3.0);
        sleep(200);
        //forward 8
        robot.encoderDrive(DRIVE_SPEED, -8, -8, 3);
        sleep(200);
        //turn left 4
        robot.encoderDrive(DRIVE_SPEED, 4, -4, 3);
        sleep(200);
        //left 13
        robot.distinctDrive(SIDE_SPEED, 13, -13, -13, 13, 3.0);
        //robot.turnToDegree(TURN_SPEED,0,imu,3);
        robot.setupGoldDetector();
        robot.goldDetector.enable();
        int moveCount=0;
        for (int i = 0; i <= 6; i++) {
            if (robot.searchForGold(1000)) {
                robot.encoderDrive(DRIVE_SPEED, -30, -30, 3.0);
                break;
            }
            robot.distinctDrive(SIDE_SPEED, -7, 7, 7, -7, 2.0);
            robot.encoderDrive(DRIVE_SPEED, -1.8, 1.8, 1);
            moveCount++;
        }
        /*
        switch(moveCount){

        }
        */


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
