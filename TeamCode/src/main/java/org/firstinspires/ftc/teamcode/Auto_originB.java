package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto B", group = "Test")
public class Auto_originB extends LinearOpMode {

    private static final double DRIVE_SPEED = .15, TURN_SPEED = 1, ARM_SPEED = .8, SIDE_SPEED = .25;

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
        sleep(200);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        robot.imu.initialize(parameters);
        sleep(1000);
        //go left 10
        robot.distinctDrive(SIDE_SPEED, 10, -10, -10, 10, 3.0);
        sleep(200);
        //turn by imu
        //turn left 6
        robot.encoderDrive(DRIVE_SPEED, 6, -6, 3);
        robot.TurnByImu(DRIVE_SPEED,0,3.0);
        sleep(1000);
        //forward 7
        robot.encoderDrive(DRIVE_SPEED, -7, -7, 3);
        sleep(200);
        //left 13
        robot.distinctDrive(SIDE_SPEED, 16, -16, -16, 16, 3.0);
        //turn left 4
        robot.encoderDrive(DRIVE_SPEED, 4, -4, 3);
        sleep(200);
        //robot.turnToDegree(TURN_SPEED,0,imu,3);
        robot.setupGoldDetector();
        robot.goldDetector.enable();
        int moveCount=0;
        for (int i = 0; i <= 7; i++) {
            if (robot.searchForGold(800)) {
                robot.encoderDrive(DRIVE_SPEED, -35, -35, 3.0);
                break;
            }
            robot.distinctDrive(SIDE_SPEED, -5, 5, 5, -5, 2.0);
            robot.encoderDrive(DRIVE_SPEED, -1.2, 1.2, 1);
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
