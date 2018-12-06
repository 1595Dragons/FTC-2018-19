package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto LandDriveForward", group = "Official")
public class Auto_LandDriveForward extends LinearOpMode {

    // Config for the robot
    private Config robot = new Config(this);


    @Override
    public void runOpMode() {

        final double ARM_SPEED = .8, SIDE_SPEED = .25, DRIVE_SPEED = .15;

        // Setup robot hardware
        robot.ConfigureRobtHardware(false);


        // Send telemetry message to signify robot waiting;
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front, robot.armMotorL, robot.armMotorR);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //PlanA
        robot.armDrive(ARM_SPEED, 660, 3);
        sleep(300);
        robot.distinctDrive(SIDE_SPEED, 10, -10, -10, 10, 3);
        sleep(300);
        robot.encoderDrive(DRIVE_SPEED, 3, -3, 3);
        sleep(300);
        robot.encoderDrive(DRIVE_SPEED, -5, -5, 2);
        robot.distinctDrive(SIDE_SPEED, -10, 10, 10, -10, 3);
        sleep(300);
        robot.encoderDrive(DRIVE_SPEED, -40, -40, 5);


    }
}
