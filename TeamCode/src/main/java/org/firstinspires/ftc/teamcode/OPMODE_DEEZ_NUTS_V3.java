package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "OPMODE_DEEZ_NUTS_V3")
public class OPMODE_DEEZ_NUTS_V3 extends LinearOpMode {


    private CRServoImplEx intake;
    private ServoImplEx linkage1;
    private ServoImplEx linkage2;
    private ServoImplEx linkage3;

    private DcMotorImplEx launcherMotorLeft;
    private DcMotorImplEx launcherMotorRight;

    private DcMotorImplEx frontRightDrive;
    private DcMotorImplEx frontLeftDrive;
    private DcMotorImplEx backRightDrive;
    private DcMotorImplEx backLeftDrive;
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;


    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    float uppos = 1;
    double downpos = 0.72;

    private final String[] pattern = {"ppg", "pgp", "gpp"};

    private int partofpattern = 0;

    private final String[][] patternindividual = {
            {"g","p","p"},
            {"p","g","p"},
            {"p","p","g"}
    };

    private String[] patternindividuallive = {"","",""};

    private int patnum = 0;



    private double abcdef = 1;

    String[] patternraw = {"a","a","a"};

    String tempcolo = "";
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
        double yaw;
        double max;

        double incrempad = 0.2;

        boolean launcheractive = false;


        intake = hardwareMap.get(CRServoImplEx.class, "intake");
        linkage1 = hardwareMap.get(ServoImplEx.class, "linkage1");
        linkage2 = hardwareMap.get(ServoImplEx.class, "linkage2");
        linkage3 = hardwareMap.get(ServoImplEx.class, "linkage3");
        launcherMotorLeft = hardwareMap.get(DcMotorImplEx.class, "launcherMotorLeft");
        launcherMotorRight = hardwareMap.get(DcMotorImplEx.class, "launcherMotorRight");
        frontLeftDrive = hardwareMap.get(DcMotorImplEx.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotorImplEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorImplEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorImplEx.class, "backRightDrive");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensorColor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensorColor2");
        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "sensorColor3");


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
        launcherMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherMotorRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);

        launcherMotorRight.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        launcherMotorLeft.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);

        final float[] hsvValues = new float[3];

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        int temppatnum = 0;
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value

            // inccrement through the array for patterns
            if (gamepad1.leftBumperWasPressed()) {
                temppatnum += 1;
                patnum = temppatnum % 3;
            }
            telemetry.addData("left bumper counter", patnum);

            //actually launch sort
            //if (gamepad1.rightBumperWasPressed()) {
            //}

            NormalizedRGBA colors1 = colorSensor1.getNormalizedColors(); Color.colorToHSV(colors1.toColor(), hsvValues);
            Color.colorToHSV(colors1.toColor(), hsvValues);
            telemetry.addData("color1", determineColor(hsvValues[0], hsvValues[1], hsvValues[2]));
            tempcolo = determineColor(hsvValues[0], hsvValues[1], hsvValues[2]);
            patternraw[0] = tempcolo;
            patternindividuallive[0] = tempcolo;

            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors(); Color.colorToHSV(colors1.toColor(), hsvValues);
            Color.colorToHSV(colors2.toColor(), hsvValues);
            telemetry.addData("color2", determineColor(hsvValues[0], hsvValues[1], hsvValues[2]));
            tempcolo = determineColor(hsvValues[0], hsvValues[1], hsvValues[2]);
            patternraw[1] = tempcolo;
            patternindividuallive[1] = tempcolo;


            NormalizedRGBA colors3 = colorSensor3.getNormalizedColors(); Color.colorToHSV(colors1.toColor(), hsvValues);
            Color.colorToHSV(colors3.toColor(), hsvValues);
            telemetry.addData("color3", determineColor(hsvValues[0], hsvValues[1], hsvValues[2]));
            tempcolo = determineColor(hsvValues[0], hsvValues[1], hsvValues[2]);
            patternraw[2] = tempcolo;
            patternindividuallive[2] = tempcolo;



            telemetry.addData("patternindividual", patternindividual[patnum][partofpattern]);
            if (gamepad1.rightBumperWasPressed()) {
                if (patternindividuallive[0] == patternindividual[patnum][partofpattern]) {
                    linkage1func(true);
                    sleep(100);
                    linkage1func(false);
                } else if (patternindividuallive[1] == patternindividual[patnum][partofpattern]) {
                    linkage2func(true);
                    sleep(100);
                    linkage2func(false);
                } else if (patternindividuallive[2] == patternindividual[patnum][partofpattern]) {
                    linkage3func(true);
                    sleep(100);
                    linkage3func(false);
                }
            }




            if (gamepad1.right_trigger == 1) {intakefunc(true);} else {intakefunc(false);}
            telemetry.addData("rt: ", gamepad1.right_trigger);

            linkage1func(gamepad1.x);
            telemetry.addData("xx: ", linkage1.getPosition());
            telemetry.addData("x: ", gamepad1.x);
            linkage2func(gamepad1.a);
            telemetry.addData("aa: ", linkage2.getPosition());
            telemetry.addData("a: ", gamepad1.a);
            linkage3func(gamepad1.b);
            telemetry.addData("bb: ", linkage3.getPosition());
            telemetry.addData("b: ", gamepad1.b);

            //if (gamepad2.dpad_up) {abcdef += 0.01;sleep(10);}
            //if (gamepad2.dpad_down) {abcdef -= 0.01;sleep(10);}
            //if (abcdef > 1) {abcdef = 1;} else if (abcdef < 0.69) {abcdef = 0.69;}
            //linkage1.setPosition(abcdef);
            //linkage2.setPosition(abcdef);
            //linkage3.setPosition(abcdef);
            //telemetry.addData("ababababababaabab", abcdef);

            if (gamepad1.right_trigger >= 0.9) {intake.setPower(1);}


            // trying release but might be test
            if (gamepad1.yWasReleased()) {launcheractive = !launcheractive;}
            telemetry.addData("launcher toggle: ", launcheractive);

            // setting launcher power
            if (launcheractive) {
                launcherMotorRight.setPower(1);
                launcherMotorLeft.setPower(1);
            } else {
                launcherMotorLeft.setPower(0);
                launcherMotorRight.setPower(0);
            }

            //move forward and back
            if (gamepad1.dpad_down || gamepad1.dpad_up) {
                telemetry.addData("up/down:  ", "true");
                //dpad up
                if (gamepad1.dpad_up) {
                    axial = incrempad;
                    telemetry.addData("up:  ", axial);
                    //dpad down \/
                } else if (gamepad1.dpad_down) {
                    axial = -incrempad;
                    telemetry.addData("down:  ", axial);
                }
            } else {
                // gamepad stick to move forwards and back
                axial = -gamepad1.left_stick_y;
                telemetry.addData("stick:  ", axial);
            }


            //drift left and right
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                telemetry.addData("left/right:  ", "true");
                //dpad left
                if (gamepad1.dpad_left) {
                    lateral = -incrempad;
                    telemetry.addData("left:  ", lateral);
                    //dpad right \/
                } else if (gamepad1.dpad_right) {
                    lateral = incrempad;
                    telemetry.addData("right:  ", lateral);
                }
            } else {
                // gamepad stick to drift
                lateral = gamepad1.left_stick_x;
                telemetry.addData("stick:  ", lateral);
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



    private void intakefunc(boolean on) {
        if (on) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }
    private void linkage1func(boolean on) {
        if (on) {
            linkage1.setPosition(uppos);
        } else {
            linkage1.setPosition(downpos);
        }
    }

    private void linkage2func(boolean on) {
        if (on) {
            linkage2.setPosition(uppos);
        } else {
            linkage2.setPosition(downpos);
        }
    }

    private void linkage3func(boolean on) {
        if (on) {
            linkage3.setPosition(uppos);
        } else {
            linkage3.setPosition(downpos);
        }
    }

    public static String determineColor(double h, double s, double v) {
        // Check if hue falls within the green range (85° to 170°)
        if (h >= 75 && h <= 185) {
            return "g";
        }
        // Check if hue falls within the purple range (260° to 320°)
        else if (h >= 210 && h <= 320) {
            return "p";
        }
        // If it's neither, return "Other"
        else {
            return "O";
        }
    }

}