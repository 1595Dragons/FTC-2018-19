package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen Ogden on 11/6/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@Autonomous(name = "Land and detect cube", group = "Official")
//@Disabled
public class detectCubeAuto extends LinearOpMode {

    private RobotConfig robot = new RobotConfig(this.telemetry);

    private int stage = 0;

    @Override
    public void runOpMode() {

        robot.configureRobot(this.hardwareMap);
        robot.resetMotors(robot.climber, robot.left1, robot.left2, robot.right1, robot.right2);

        ElapsedTime time = new ElapsedTime();

        waitForStart();
        while (opModeIsActive()) {

            switch (stage) {
                case 0:
                    robot.climber.setTargetPosition(robot.maxClimberPos);
                    robot.climber.setPower(1);
                    if (robot.isThere(5, robot.climber)) {
                        robot.climber.setPower(0);
                        stage++;
                    }
                    break;
                case 1:
                    robot.driveDistance(MecanumDriveDirection.RIGHT, 4, 1);
                    if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 2:
                    robot.climber.setTargetPosition(robot.minClimberPos);
                    robot.climber.setPower(1);
                    if (robot.isThere(5, robot.climber)) {
                        robot.climber.setPower(0);
                        stage++;
                    }
                    break;
                case 3:
                    robot.driveDistance(MecanumDriveDirection.FORWARD, 12, 1);
                    if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 4:
                    robot.driveDistance(MecanumDriveDirection.LEFT, 3, 1);
                    if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 5:
                    time.reset();
                    robot.setupGoldDetector(this.hardwareMap);
                    stage++;
                    break;
                case 6:
                    if (robot.goldDetector.isFound()) {
                        robot.goldDetector.disable();
                        stage = 11;
                    } else if (time.seconds() > 3) {
                        robot.goldDetector.disable();
                        stage++;
                    }
                    break;
                case 7:
                    robot.driveDistance(MecanumDriveDirection.LEFT, 16, 1);
                    if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 8:
                    time.reset();
                    robot.goldDetector.enable();
                    stage++;
                    break;
                case 9:
                    if (robot.goldDetector.isFound()) {
                        robot.goldDetector.disable();
                        stage = 11;
                    } else if (time.seconds() > 3) {
                        robot.goldDetector.disable();
                        stage++;
                    }
                    break;
                case 10:
                    robot.driveDistance(MecanumDriveDirection.RIGHT, 32, 1);
                    if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
                case 11:
                    robot.driveDistance(MecanumDriveDirection.FORWARD, 36, 1);
                    if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                        robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                        stage++;
                    }
                    break;
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("Time", time.seconds());
            telemetry.addLine();
            robot.updateTelemetry();

            idle();

        }

    }
}
