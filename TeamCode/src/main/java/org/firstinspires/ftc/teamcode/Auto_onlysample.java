package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto OnlySample", group = "Test")
public class Auto_onlysample extends LinearOpMode {

    private static final double DRIVE_SPEED = .2, TURN_SPEED = 1, ARM_SPEED = 1, SIDE_SPEED = .2;

    // Config for the robot
    private Config robot = new Config(this);


    @Override
    public void runOpMode() {

        // Setup robot hardware
        robot.ConfigureRobtHardware(true);


        // Send telemetry message to signify robot waiting;
        robot.status("Resetting motors");
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front, robot.armMotorL, robot.armMotorR);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //PlanA
        robot.armDrive(ARM_SPEED, 680, 3);
        //go left 10
        robot.distinctDrive(SIDE_SPEED, 10, -10, -10, 10, 3.0);
        //turn by imu
        sleep(200);
        robot.TurnByImu(DRIVE_SPEED,0,2.0);
        sleep(200);
        //forward 7
        robot.encoderDrive(DRIVE_SPEED, -5, -5, 3);
        //left 13
        robot.distinctDrive(SIDE_SPEED, 9, -9, -9, 9, 2.0);
        robot.distinctDrive(SIDE_SPEED, 8, -8, -8, 8, 2.0);
        sleep(200);
        robot.TurnByImu(DRIVE_SPEED,0,2.0);
        sleep(200);
        //robot.turnToDegree(TURN_SPEED,0,imu,3);
        robot.setupGoldDetector();
        robot.goldDetector.enable();
        sleep(1000);
        int moveCount=0;
        for (int i = 0; i <= 7; i++) {
            if (robot.searchForGold(500)) {
                robot.encoderDrive(0.15, -20, -20, 3.0);
                break;
            }
            robot.distinctDrive(SIDE_SPEED, -6, 6, 6, -6, 2.0);
            robot.TurnByImu(DRIVE_SPEED,0,1.0);
            moveCount++;
        }
        if (moveCount>=8)
        {
            robot.encoderDrive(0.15, -35, -35, 3.0);
        }

        


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
