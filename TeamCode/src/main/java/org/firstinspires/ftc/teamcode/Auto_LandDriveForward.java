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
        robot.resetMotors(robot.left_back, robot.left_front, robot.right_back, robot.right_front, robot.armMotorL, robot.armMotorR);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //PlanA
        robot.armDrive(ARM_SPEED, 680, 3.0);
        sleep(300);
        robot.distinctDrive(SIDE_SPEED, 10, -10, -10, 10, 3);
        sleep(300);
        robot.encoderDrive(DRIVE_SPEED, 3, -3, 3);
        sleep(300);
        robot.encoderDrive(DRIVE_SPEED, -5, -5, 2);
        robot.distinctDrive(SIDE_SPEED, -10, 10, 10, -10, 3);
        sleep(300);
        robot.encoderDrive(DRIVE_SPEED, -40, -40, 5);


        // Reset the runtime
        runtime.reset();
        while(runtime.seconds()<timeoutS && ((angles2.firstAngle-turnToAngle)>=1||(angles2.firstAngle-turnToAngle)<=-1))
        {
            angles2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if ((lastDifference>0&&(angles2.firstAngle-turnToAngle)<=0)
                    ||((lastDifference<0&&(angles2.firstAngle-turnToAngle)>=0)))
            {
                accumulation=0;
            }
            accumulation+=(runtime.seconds()-lastTime)*(angles2.firstAngle-turnToAngle);
            output=kp*(angles2.firstAngle-turnToAngle)+ki*accumulation+kd*(angles2.firstAngle-lastAngle)/(runtime.seconds()-lastTime);
            lastTime=runtime.seconds();
            lastAngle=angles2.firstAngle;
            lastDifference=angles2.firstAngle-turnToAngle;
            robot.left_front.setPower(output*speed);
            robot.left_back.setPower(output*speed);
            robot.right_front.setPower(-output*speed);
            robot.right_back.setPower(-output*speed);
        }
        // Stop all motion;
        robot.left_front.setPower(0);
        robot.left_back.setPower(0);
        robot.right_front.setPower(0);
        robot.right_back.setPower(0);
    }
}
