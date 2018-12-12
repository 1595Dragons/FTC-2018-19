package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Stephen Ogden on 12/7/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@Autonomous(name = "Drive sideways", group = "Test")
public class DriveSidewaysTest extends LinearOpMode {
    // Config for the robot
    private Config robot = new Config(this);

    @Override
    public void runOpMode() {

        // Setup robot hardware
        robot.ConfigureRobtHardware(true);
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.autoDriveSideways(.25d, 24, robot.getAngle(), 10);
        sleep(1000);
        robot.autoDriveSideways(.25d, -24, robot.getAngle(), 10);
        sleep(1000);

    }
}
