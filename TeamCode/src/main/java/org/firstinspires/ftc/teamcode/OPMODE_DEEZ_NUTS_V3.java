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

import java.util.concurrent.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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
    double downpos = 0.70;

    double servo_shift_pos_up = 0.76;

    // Distance threshold (cm) - ball is considered present if closer than this
    // distance 6, 1.5,
    double BALL_PRESENT_DISTANCE_CM = 4.0;

    private final String[] pattern = {"gpp", "pgp", "ppg"};
    private final String[] fullbar_color_pattern = {
            "🟩🟩🟩🟩🟩🟪🟪🟪🟪🟪🟪🟪🟪🟪🟪",
            "🟪🟪🟪🟪🟪🟩🟩🟩🟩🟩🟪🟪🟪🟪🟪",
            "🟪🟪🟪🟪🟪🟪🟪🟪🟪🟪🟩🟩🟩🟩🟩"
    };

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

    private ElapsedTime jiggleTimer = new ElapsedTime();
    private ElapsedTime linkage1ColorDetectedTimer = new ElapsedTime();
    private ElapsedTime linkage2ColorDetectedTimer = new ElapsedTime();
    private ElapsedTime linkage3ColorDetectedTimer = new ElapsedTime();

    private final double COLOR_DETECTED_HOLD_TIME = 2.0; // seconds to keep servo down after color detected

    // Track if each position has a ball with known color (locked until launched)
    private boolean ballLocked1 = false;
    private boolean ballLocked2 = false;
    private boolean ballLocked3 = false;

    // Store the locked color to detect if ball moved/changed
    private String lockedColor1 = "";
    private String lockedColor2 = "";
    private String lockedColor3 = "";



    // GAMEPAD 2 TOGGLE DEBUG
    boolean jiggle_lb = false;
    boolean launcher_toggle_y = false;
    boolean stick_rightstickbutton = false;
    boolean dpad_dpadup = false;
    boolean movement_leftstickbutton = false;
    boolean righttrigger_rt = false;
    boolean rightbumper_rb = false;
    boolean color_distance_x = false;
    boolean locked_a = false;
    boolean patterndisp_b = false;
    boolean toggleall_lt = false;
    boolean colorpattern = true;
    boolean temp = false;

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
        boolean launcheroff = false;


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
            // Note: pushing stick forward gives negative value


            // GAMEPAD 2 DEBUGING
            if (gamepad2.leftBumperWasReleased() || toggleall_lt) {
                jiggle_lb = !jiggle_lb;
                temp = true;
            }
            if (gamepad2.yWasReleased() || toggleall_lt) {
                launcher_toggle_y = !launcher_toggle_y;
                temp = true;
            }
            if (gamepad2.rightStickButtonWasReleased() || toggleall_lt) {
                stick_rightstickbutton = !stick_rightstickbutton;
                temp = true;
            }
            if (gamepad2.dpadUpWasReleased() || toggleall_lt) {
                dpad_dpadup = !dpad_dpadup;
                temp = true;
            }
            if (gamepad2.leftStickButtonWasReleased() || toggleall_lt) {
                movement_leftstickbutton = !movement_leftstickbutton;
                temp = true;
            }
            if (gamepad2.dpadRightWasReleased() || toggleall_lt) {
                righttrigger_rt = !righttrigger_rt;
                temp = true;
            }
            if (gamepad2.rightBumperWasReleased() || toggleall_lt) {
                rightbumper_rb = !rightbumper_rb;
                temp = true;
            }
            if (gamepad2.xWasReleased() || toggleall_lt) {
                color_distance_x = !color_distance_x;
                temp = true;
            }
            if (gamepad2.aWasReleased() || toggleall_lt) {
                locked_a = !locked_a;
                temp = true;
            }
            if (gamepad2.bWasReleased() || toggleall_lt) {
                patterndisp_b = !patterndisp_b;
                temp = true;
            }
            if (gamepad2.dpadLeftWasReleased() || toggleall_lt) {
                toggleall_lt = !toggleall_lt;
                temp = true;
            }
            if (temp){
                colorpattern = !colorpattern;
                temp = false;
            }


            // inccrement through the array for patterns
            if (gamepad1.leftBumperWasPressed()) {
                temppatnum += 1;
                patnum = temppatnum % 3;
                partofpattern = 0;
            }
            // right bumper
            if (rightbumper_rb) {
                telemetry.addData("right bumper counter (patnum)", patnum);
            }

            //actually launch sort
            //if (gamepad1.rightBumperWasPressed()) {
            //}

            // Color Sensor 1 - Check distance and color
            NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
            Color.colorToHSV(colors1.toColor(), hsvValues);
            double distance1 = ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM);
            // color distance
            if (color_distance_x) {
                telemetry.addData("color1", determineColor(hsvValues[0], hsvValues[1], hsvValues[2]));
                telemetry.addData("distance1", distance1);
            }
            tempcolo = determineColor(hsvValues[0], hsvValues[1], hsvValues[2]);
            patternraw[0] = tempcolo;
            patternindividuallive[0] = tempcolo;

            // Lock ball 1 if ball present (close distance) AND color detected
            if (distance1 < BALL_PRESENT_DISTANCE_CM && !tempcolo.equals("O")) {
                if (!ballLocked1) {
                    ballLocked1 = true;
                    lockedColor1 = tempcolo;
                }
                linkage1ColorDetectedTimer.reset();
            } else if (distance1 >= BALL_PRESENT_DISTANCE_CM) {
                // Ball is gone - unlock
                ballLocked1 = false;
                lockedColor1 = "";
            }

            // Color Sensor 2 - Check distance and color
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
            Color.colorToHSV(colors2.toColor(), hsvValues);
            double distance2 = ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM);
            if (color_distance_x) {
                telemetry.addData("color2", determineColor(hsvValues[0], hsvValues[1], hsvValues[2]));
                telemetry.addData("distance2", distance2);
            }
            tempcolo = determineColor(hsvValues[0], hsvValues[1], hsvValues[2]);
            patternraw[1] = tempcolo;
            patternindividuallive[1] = tempcolo;

            // Lock ball 2 if ball present (close distance) AND color detected
            if (distance2 < BALL_PRESENT_DISTANCE_CM && !tempcolo.equals("O")) {
                if (!ballLocked2) {
                    ballLocked2 = true;
                    lockedColor2 = tempcolo;
                }
                linkage2ColorDetectedTimer.reset();
            } else if (distance2 >= BALL_PRESENT_DISTANCE_CM) {
                // Ball is gone - unlock
                ballLocked2 = false;
                lockedColor2 = "";
            }

            // Color Sensor 3 - Check distance and color
            NormalizedRGBA colors3 = colorSensor3.getNormalizedColors();
            Color.colorToHSV(colors3.toColor(), hsvValues);
            double distance3 = ((DistanceSensor) colorSensor3).getDistance(DistanceUnit.CM);
            if (color_distance_x) {
                telemetry.addData("color3", determineColor(hsvValues[0], hsvValues[1], hsvValues[2]));
                telemetry.addData("distance3", distance3);
            }
            tempcolo = determineColor(hsvValues[0], hsvValues[1], hsvValues[2]);
            patternraw[2] = tempcolo;
            patternindividuallive[2] = tempcolo;

            // Lock ball 3 if ball present (close distance) AND color detected
            if (distance3 < BALL_PRESENT_DISTANCE_CM && !tempcolo.equals("O")) {
                if (!ballLocked3) {
                    ballLocked3 = true;
                    lockedColor3 = tempcolo;
                }
                linkage3ColorDetectedTimer.reset();
            } else if (distance3 >= BALL_PRESENT_DISTANCE_CM) {
                // Ball is gone - unlock
                ballLocked3 = false;
                lockedColor3 = "";
            }

            // Display lock status
            // locked
            if (locked_a) {
                telemetry.addData("Ball Locked 1", ballLocked1 ? lockedColor1 : "No");
                telemetry.addData("Ball Locked 2", ballLocked2 ? lockedColor2 : "No");
                telemetry.addData("Ball Locked 3", ballLocked3 ? lockedColor3 : "No");
            }


            int sleep_time_auto = 500;
            // pattern disp
            if (patterndisp_b) {
                telemetry.addData("full pattern", pattern[patnum]);
                telemetry.addData("patternindividual (current color to launch): ", patternindividual[patnum][partofpattern]);
                telemetry.addData("partofpattern: ", partofpattern);
            }

            if (colorpattern) {
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
                telemetry.addLine(fullbar_color_pattern[patnum]);
            }

            if (gamepad1.rightBumperWasPressed()) {
                if (patternindividuallive[0] == patternindividual[patnum][partofpattern]) {
                    linkage1func(true);
                    sleep(sleep_time_auto);
                    linkage1func(false);
                    partofpattern = (partofpattern + 1) % 3;
                    sleep(50);
                } else if (patternindividuallive[1] == patternindividual[patnum][partofpattern]) {
                    linkage2func(true);
                    sleep(sleep_time_auto);
                    linkage2func(false);
                    partofpattern = (partofpattern + 1) % 3;
                    sleep(50);
                } else if (patternindividuallive[2] == patternindividual[patnum][partofpattern]) {
                    linkage3func(true);
                    sleep(sleep_time_auto);
                    linkage3func(false);
                    partofpattern = (partofpattern + 1) % 3;
                    sleep(50);
                }
                sleep(sleep_time_auto);
            }


            if (gamepad1.right_trigger == 1) {
                intakefunc(true);
            } else {
                intakefunc(false);
            }
            // right trigger
            if (righttrigger_rt) {
                telemetry.addData("Right Trigger: ", gamepad1.right_trigger);
            }


            // 1. Check for Manual Overrides (X, A, or B) or Intake Running
            boolean manualControl = gamepad1.x || gamepad1.a || gamepad1.b;
            boolean intakeRunning = gamepad1.right_trigger >= 0.9;

            if (intakeRunning) {
                // Intake is running - keep all servos DOWN so balls can enter
                linkage1.setPosition(downpos);
                linkage2.setPosition(downpos);
                linkage3.setPosition(downpos);
            } else if (!manualControl) {
                // 2. If ball is locked (confirmed present with known color), keep servo DOWN and DON'T jiggle
                // Keep servos down if ball is locked
                if (ballLocked1) linkage1.setPosition(downpos);
                if (ballLocked2) linkage2.setPosition(downpos);
                if (ballLocked3) linkage3.setPosition(downpos);

                // Now handle jiggling ONLY for unlocked positions
                double time = jiggleTimer.seconds();

                if (time < 1.0) {
                    // --- PHASE 1: Jiggle UP (First 1 second) ---
                    if (!ballLocked1) linkage1.setPosition(servo_shift_pos_up);
                    if (!ballLocked2) linkage2.setPosition(servo_shift_pos_up);
                    if (!ballLocked3) linkage3.setPosition(servo_shift_pos_up);
                    // jiggle
                    if (jiggle_lb){telemetry.addData("Jiggle Phase","UP");}
                }
                else if (time < 2.0) {
                    // --- PHASE 2: Jiggle DOWN (Next 1 second) ---
                    if (!ballLocked1) linkage1.setPosition(downpos);
                    if (!ballLocked2) linkage2.setPosition(downpos);
                    if (!ballLocked3) linkage3.setPosition(downpos);
                    // jiggle
                    if (jiggle_lb){telemetry.addData("Jiggle Phase","DOWN");}
                }
                else {
                    // --- PHASE 3: Reset Cycle (After 2 seconds total) ---
                    jiggleTimer.reset();
                }
            } else {
                // 3. Manual Override logic (Your existing X, A, B buttons)
                linkage1func(gamepad1.x);
                linkage2func(gamepad1.a);
                linkage3func(gamepad1.b);
            }


            //if (gamepad2.dpad_up) {abcdef += 0.01;sleep(10);}
            //if (gamepad2.dpad_down) {abcdef -= 0.01;sleep(10);}
            //if (abcdef > 1) {abcdef = 1;} else if (abcdef < 0.69) {abcdef = 0.69;}
            //linkage1.setPosition(abcdef);
            //linkage2.setPosition(abcdef);
            //linkage3.setPosition(abcdef);
            //telemetry.addData("ababababababaabab", abcdef);

            if (gamepad1.right_trigger >= 0.9) {
                intake.setPower(1);
            }


            // Launcher always runs at low speed to stay warmed up
            // Y button can still toggle full speed if needed
            if (gamepad1.yWasReleased()) {
                launcheractive = !launcheractive;
            }
            if (gamepad1.rightStickButtonWasReleased()) {
                launcheroff = !launcheroff;
            }


            // launchertoggle
            if (launcher_toggle_y){telemetry.addData("launcher full speed: ", launcheractive);}

            // setting launcher power - always at least 0.3, full speed if toggled
            double launcher_percent = 0.70;
            if (gamepad1.left_stick_button) {
                launcherMotorLeft.setPower(-0.1);
                launcherMotorRight.setPower(-0.1);
            } else {
                if (launcheractive) {
                    launcherMotorRight.setPower(launcher_percent);
                    launcherMotorLeft.setPower(launcher_percent);
                } else {
                    if (!launcheroff) {
                        launcherMotorLeft.setPower(0.3);
                        launcherMotorRight.setPower(0.3);
                    } else {
                        launcherMotorLeft.setPower(0);
                        launcherMotorRight.setPower(0);
                    }

                }
            }

            //move forward and back
            if (gamepad1.dpad_down || gamepad1.dpad_up) {
                if (gamepad1.dpad_up) {
                    axial = incrempad;
                    // dpad
                    if (dpad_dpadup){telemetry.addData("up:  ", axial);}
                    //dpad down \/
                } else if (gamepad1.dpad_down) {
                    axial = -incrempad;
                    if (dpad_dpadup){telemetry.addData("down:  ", axial);}
                }
            } else {
                // gamepad stick to move forwards and back
                axial = -gamepad1.left_stick_y;
                // stick
                if (stick_rightstickbutton){telemetry.addData("stick:  ", axial);}
            }


            //drift left and right
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                if (gamepad1.dpad_left) {
                    lateral = -incrempad;
                    // dpad
                    if (dpad_dpadup){telemetry.addData("left:  ", lateral);}
                    //dpad right \/
                } else if (gamepad1.dpad_right) {
                    lateral = incrempad;
                    if (dpad_dpadup) {telemetry.addData("right:  ", lateral);}
                }
            } else {
                // gamepad stick to drift OR SOMTHING
                lateral = gamepad1.left_stick_x;
                // stick
                if (stick_rightstickbutton){telemetry.addData("stick:  ", lateral);}
            }

            yaw = gamepad1.right_stick_x;
            // Combine the ADNUEOAYWIMBYAXNIXONAENOADSIU stseuqer for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save DAIUGOWGDMYVQIDQYIUADNHS power level for telemetry.
            frontLeftPower = axial + lateral + yaw;
            frontRightPower = (axial - lateral) - yaw;
            backLeftPower = (axial - lateral) + yaw;
            backRightPower = (axial + lateral) - yaw;
            // Normalize the values so no wheel power exceeds 100% REEEEERAIUAKDHWOADNHOMQXNJW
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
            // Always enabled
            telemetry.addData("Status", "Run Time: " + runtime);
            // movement
            if (movement_leftstickbutton) {
                telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
                telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));
            }
            telemetry.update();
        }

    }

    /**
     * EEEEEEEe3E3EE#EYE^
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
        // Check if hue falls within the green range (85° to 170°) FUCKFFUFUFFUUFUFUFFFFFFF
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
