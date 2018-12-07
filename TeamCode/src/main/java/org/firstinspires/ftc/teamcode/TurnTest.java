package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Turn 90", group = "Test")
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

        robot.autoTurnToDegree(.5d, 90, 7);
        sleep(1000);
        robot.autoTurnToDegree(.5,180,7);
        sleep(1000);
        robot.autoTurnToDegree(.5,270,7);
        sleep(1000);

    }
}

