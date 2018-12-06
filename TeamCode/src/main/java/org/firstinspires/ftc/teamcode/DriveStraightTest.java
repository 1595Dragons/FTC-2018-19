package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Drive straight", group = "Test")
public class DriveStraightTest extends LinearOpMode {

    // Config for the robot
    private Config robot = new Config(this);

    @Override
    public void runOpMode() {

        // Setup robot hardware
        robot.ConfigureRobtHardware(true);
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.autoDriveForward(.5d, 12, 10, Math.round(robot.getAngles().secondAngle));
        sleep(1000);
        robot.autoDriveForward(.5d, -12, 10, Math.round(robot.getAngles().secondAngle));

    }
}

