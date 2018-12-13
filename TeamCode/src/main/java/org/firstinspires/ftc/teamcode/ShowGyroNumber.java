package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ShowAngle", group = "Test")
public class ShowGyroNumber extends LinearOpMode {

    // Config for the robot
    private Config robot = new Config(this);

    @Override
    public void runOpMode() {

        // Setup robot hardware
        robot.ConfigureRobtHardware(true);
        robot.resetMotorsForAutonomous(robot.left_back, robot.left_front, robot.right_back, robot.right_front);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        robot.imu.initialize(parameters);
        while (getRuntime()<=30)
        {
            telemetry.addData("angle",robot.getAngle());
            telemetry.addData("show error", robot.getError(0));
            telemetry.update();
        }
        //robot.TurnByImu(0.15, 180,3.0);
    }


}

