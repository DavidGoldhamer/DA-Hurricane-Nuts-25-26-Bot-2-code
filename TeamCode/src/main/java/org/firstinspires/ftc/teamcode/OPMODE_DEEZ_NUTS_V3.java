package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "OPMODE_DEEZ_NUTS_V3")
public class OPMODE_DEEZ_NUTS_V3 extends LinearOpMode {

    ////////////private DcMotorImplEx launcherMotorLeft;
    ///////////private DcMotorImplEx launcherMotorRight;

    private DcMotorImplEx frontRightDrive;
    private DcMotorImplEx frontLeftDrive;
    private DcMotorImplEx backRightDrive;
    private DcMotorImplEx backLeftDrive;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        double axial = 0;
        double lateral = 0;
        float yaw;
        double max;

        //////launcherMotorLeft = hardwareMap.get(DcMotorImplEx.class, "launcherMotorLeft");
        ////////launcherMotorRight = hardwareMap.get(DcMotorImplEx.class, "launcherMotorRight");
        frontLeftDrive = hardwareMap.get(DcMotorImplEx.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotorImplEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorImplEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorImplEx.class, "backRightDrive");


        runtime = new ElapsedTime();
        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        //
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot
        // (the wheels turn the same direction as the motor shaft).
        //
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        //
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        // <--- Click blue icon to see important note re. testing motor directions.
        /////////launcherMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        /////////launcherMotorRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            double incrempad = 0.1;

            //move forward and back
            if (gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) {
                //dpad up
                if (gamepad1.dpadUpWasPressed()) {
                    axial = incrempad;
                    //dpad down \/
                } else if (gamepad1.dpadDownWasPressed()) {
                    axial = -incrempad;
                }
            } else {
                // gamepad stick to move forwards and back
                axial = -gamepad1.left_stick_y;
            }


            //drift left and right
            if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
                //dpad left
                if (gamepad1.dpadLeftWasPressed()) {
                    lateral = incrempad;
                    //dpad right \/
                } else if (gamepad1.dpadRightWasPressed()) {
                    lateral = -incrempad;
                }
            } else {
                // gamepad stick to drift
                lateral = gamepad1.left_stick_x;
            }

            yaw = gamepad1.right_stick_x;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            frontLeftPower = axial + lateral + yaw;
            frontRightPower = (axial - lateral) - yaw;
            backLeftPower = (axial - lateral) + yaw;
            backRightPower = (axial + lateral) - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (max > 1) {
                frontLeftPower = frontLeftPower / max;
                frontRightPower = frontRightPower / max;
                backLeftPower = backLeftPower / max;
                backRightPower = backRightPower / max;
            }
            // Send calculated power to wheels.
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
            telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));
            telemetry.update();
        }
    }

    /**
     * This function is used to test your motor directions.
     *
     * Each button should make the corresponding motor run FORWARD.
     *
     *   1) First get all the motors to take to correct positions on the robot
     *      by adjusting your Robot Configuration if necessary.
     *
     *   2) Then make sure they run in the correct direction by modifying the
     *      the setDirection() calls above.
     */
    private void testMotorDirections() {
        frontLeftPower = gamepad1.x ? 1 : 0;
        backLeftPower = gamepad1.a ? 1 : 0;
        frontRightPower = gamepad1.y ? 1 : 0;
        backRightPower = gamepad1.b ? 1 : 0;
    }
}