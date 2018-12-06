package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Turn to degree test", group = "Test")
public class TurnTest extends LinearOpMode {

    // Config for the robot
    private Config robot = new Config(this);

    @Override
    public void runOpMode() {

        // Setup robot hardware
        robot.ConfigureRobtHardware(true);
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.turnToDegree(.5, 90, robot.imu, 7);
        sleep(1000);
        robot.turnToDegree(.5, 179, robot.imu, 7);
        sleep(1000);
        robot.turnToDegree(.5, -90, robot.imu, 7);
        sleep(1000);
        robot.turnToDegree(.5, -1, robot.imu, 7);

    }
}

