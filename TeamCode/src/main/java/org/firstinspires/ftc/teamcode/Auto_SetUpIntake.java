package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//try to set up the intake in auto
@Disabled
@Autonomous(name = "SetUpIntake", group = "Official")
public class Auto_SetUpIntake extends LinearOpMode {

    private static final double DRIVE_SPEED = .2, TURN_SPEED = 1, ARM_SPEED = .8, SIDE_SPEED = .25, ARM_DROP = .3;


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
        robot.armDrive(ARM_SPEED, 680, 4);
        sleep(200);
        robot.distinctDrive(SIDE_SPEED, 10, -10, -10, 10, 4.0);
        sleep(200);
        robot.armDrive(ARM_SPEED, -350, 3);
        sleep(200);
        robot.armMotorL.setPower(-ARM_DROP);
        robot.armMotorR.setPower(-ARM_DROP);
        robot.encoderDrive(DRIVE_SPEED * 0.5, -20, -20, 5);
        robot.armMotorL.setPower(0);
        robot.armMotorR.setPower(0);

        robot.armDrive(ARM_SPEED, 500, 4);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
