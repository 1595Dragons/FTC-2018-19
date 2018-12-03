package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Imu Test", group = "Test")
public class Auto_ImuTest extends LinearOpMode {

    private static final double DRIVE_SPEED = .2, TURN_SPEED = .35, ARM_SPEED = .8, SIDE_SPEED = .25;

    // Config for the robot
    private Config robot = new Config(this);

    @Override
    public void runOpMode() {

        // Setup robot hardware
        robot.ConfigureRobtHardware(true);
        robot.resetMotors(robot.left_back, robot.left_front, robot.right_back, robot.right_front, robot.armMotorL, robot.armMotorR);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        robot.distinctDrive(SIDE_SPEED, 9, -9, -9, 9, 4.0);
        robot.turnToDegree(TURN_SPEED, 0, robot.imu, 8);
        while (opModeIsActive()) {
            robot.updateTelemetry();
        }

        //turnToDegree(TURN_SPEED,90,8);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}

