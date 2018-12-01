package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen Ogden on 11/30/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@TeleOp(name = "6128 TeleOp (Only 1 controller)")
public class TeleOpSingleController extends LinearOpMode {
        // Declare the RobotConfig file, that way we can use the pre-made fictions for cleaner code
        private RobotConfig robot = new RobotConfig(this.telemetry);

        public void runOpMode() {

            // Initialize the robot
            robot.configureRobot(this.hardwareMap);

            int armStage = 0;
            boolean moveArmUp = false, moveArmDown = false;

            // Wait for the start button to be pressed
            waitForStart();

            while (opModeIsActive()) {

                // Setup a variable for each drive wheel
                double driveForward = gamepad1.left_stick_y,
                        driveRightSide = gamepad1.left_stick_x,
                        turnRight = -1*gamepad1.right_stick_x;


                // Calculate the power value and then send power to motors
                robot.left1.setPower(Range.clip((-driveRightSide + driveForward + turnRight), -1.0, 1.0));
                robot.right1.setPower(Range.clip((driveRightSide + driveForward - turnRight), -1.0, 1.0));
                robot.left2.setPower(Range.clip((driveRightSide + driveForward + turnRight), -1.0, 1.0));
                robot.right2.setPower(Range.clip((-driveRightSide + driveForward - turnRight), -1.0, 1.0));


                // Climb to predefined positions
                if (gamepad1.y) {
                    robot.climber.setTargetPosition(robot.maxClimberPos);
                } else if (gamepad1.x) {
                    robot.climber.setTargetPosition(robot.minClimberPos);
                }
                if (robot.isThere(1, robot.climber)) {
                    robot.climber.setPower(0);
                } else {
                    robot.climber.setPower(1);
                }

                // Set arm positions
                if (gamepad1.left_bumper) {
                    // Lower
                    moveArmUp = false;
                    moveArmDown = true;
                    if (armStage == 4) {
                        armStage = 3;
                    }

                } else if (gamepad1.right_bumper) {
                    // Raise
                    moveArmDown = false;
                    moveArmUp = true;
                    if (armStage == 0) {
                        armStage = 1;
                    }
                }


                // Arm movement
                switch (armStage) {
                    case 0:
                        moveArmDown = false;
                        moveArmUp = false;
                        break;
                    case 1:
                        if (moveArmUp) {
                            robot.arm.setTargetPosition(robot.adjustedArmPos);
                        } else if (moveArmDown) {
                            robot.arm.setTargetPosition(robot.minArmPos);
                        }

                        if (robot.isThere(5, robot.arm)) {
                            robot.arm.setPower(0);
                            if (moveArmUp) {
                                armStage = 2;
                            } else if (moveArmDown) {
                                armStage = 0;
                            }
                        } else {
                            robot.arm.setPower(0.6);
                        }

                        break;
                    case 2:
                        if (moveArmUp) {
                            robot.climber.setTargetPosition(robot.maxClimberPos);
                        } else if (moveArmDown) {
                            robot.climber.setTargetPosition(robot.minArmPos);
                        }

                        if (robot.isThere(5, robot.climber)) {
                            robot.climber.setPower(0);
                            if (moveArmUp) {
                                armStage = 3;
                            } else if (moveArmDown) {
                                armStage = 1;
                            }
                        } else {
                            robot.climber.setPower(1);
                        }

                        break;
                    case 3:
                        if (moveArmUp) {
                            robot.arm.setTargetPosition(robot.topArmPos);
                        } else if (moveArmDown) {
                            robot.arm.setTargetPosition(robot.adjustedArmPos);
                        }

                        if (robot.isThere(5, robot.arm)) {
                            robot.arm.setPower(0);
                            if (moveArmUp) {
                                armStage = 4;
                            } else if (moveArmDown) {
                                armStage = 2;
                            }
                        } else {
                            robot.arm.setPower(1);
                        }
                        break;
                    case 4:
                        moveArmDown = false;
                        moveArmUp = false;
                        break;
                }

                // Set the intake to the power left trigger compared to the right trigger
                robot.intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);


            /*
            // Set the arm power to that of the left stick, but cap it at 60%
            if (Math.abs(robot.climber.getCurrentPosition() - robot.maxClimberPos) < 100) {
                robot.arm.setPower(Range.clip(gamepad1.left_stick_y, -0.6, 0.6));
            } else {
                robot.arm.setPower(0);
            }
            */


                // Update telemetry
                telemetry.addData("Arm stage", armStage);
                telemetry.addLine();
                robot.updateTelemetry();

            }

            // Stop tracking gold
            robot.goldDetector.disable();

        }
}
