package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Blue Auto", group = "Blue")
public class Auto_blue extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    // AprilTag variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Hardware
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

    // Servo positions
    float uppos = 1;
    double downpos = 0.70;

    // Distance threshold for ball detection
    double BALL_PRESENT_DISTANCE_CM = 4.0;

    public static final String PATTERN_KEY = "PATTERN_KEY";

    Object PATTERN_BOARD = blackboard.getOrDefault(PATTERN_KEY, 0);

    // Pattern definitions
    private final String[] pattern = {"gpp", "pgp", "ppg"};
    private final String[][] patternindividual = {
            {"g","p","p"},  // Pattern 0 (AprilTag ID 21)
            {"p","g","p"},  // Pattern 1 (AprilTag ID 22)
            {"p","p","g"}   // Pattern 2 (AprilTag ID 23)
    };

    private String[] patternindividuallive = {"","",""};
    private int patnum = -1; // -1 means no pattern detected yet
    private int partofpattern = 0;
    private boolean patternDetected = false;

    @Override
    public void runOpMode() {
        // Initialize AprilTag
        initAprilTag();

        // Initialize hardware
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

        // Set motor directions
        launcherMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherMotorRight.setDirection(DcMotor.Direction.REVERSE);
        launcherMotorRight.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        launcherMotorLeft.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);

        // Set all linkages to down position initially
        linkage1.setPosition(downpos);
        linkage2.setPosition(downpos);
        linkage3.setPosition(downpos);

        final float[] hsvValues = new float[3];

        telemetry.addData("Status", "Initialized - Waiting for AprilTag");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // PHASE 1: Detect AprilTag pattern
            telemetry.addData("Phase", "Detecting AprilTag pattern...");
            telemetry.update();

            while (opModeIsActive() && !patternDetected) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        // Map AprilTag ID to pattern number
                        // ID 21 -> pattern 0 (gpp), ID 22 -> pattern 1 (pgp), ID 23 -> pattern 2 (ppg)
                        if (detection.id == 21) {
                            patnum = 0;
                            patternDetected = true;
                        } else if (detection.id == 22) {
                            patnum = 1;
                            patternDetected = true;
                        } else if (detection.id == 23) {
                            patnum = 2;
                            patternDetected = true;
                        }

                        blackboard.put(PATTERN_KEY, patnum);

                        if (patternDetected) {
                            telemetry.addData("AprilTag Detected", "ID %d", detection.id);
                            telemetry.addData("Pattern Set", patnum);
                            telemetry.update();
                            sleep(500); // Brief pause to show detection
                            break;
                        }
                    }
                }

                telemetry.addData("Status", "Scanning for AprilTag...");
                telemetry.addData("Tags detected", currentDetections.size());
                telemetry.update();
                sleep(50);
            }

            if (patternDetected) {
                strafeRight(100,100);
                sleep(100);
                rotateCounterClockwise(100, 115);
            }

            // PHASE 2: Jiggle to settle balls
            if (patternDetected) {
                telemetry.addData("Phase", "Jiggling to settle balls...");
                telemetry.update();

                // Jiggle cycle to settle balls (7 full cycles)
//                for (int i = 0; i < 6; i++) {
//                    // Jiggle UP
//                    linkage1.setPosition(0.76);
//                    linkage2.setPosition(0.76);
//                    linkage3.setPosition(0.76);
//                    sleep(250);
//
//                    // Jiggle DOWN
//                    linkage1.setPosition(downpos);
//                    linkage2.setPosition(downpos);
//                    linkage3.setPosition(downpos);
//                    sleep(900);
//                }

                // PHASE 3: Spin up launcher
                telemetry.addData("Phase", "Spinning up launcher...");
                telemetry.update();

                double launcher_power = 0.67;
                launcherMotorLeft.setPower(launcher_power);
                launcherMotorRight.setPower(launcher_power);
                sleep(2000); // Give launcher time to spin up

                // PHASE 4: Detect ball colors and launch in order
                telemetry.addData("Phase", "Launching balls in order...");
                telemetry.update();

                int sleep_time_auto = 500;

                // Ensure all servos are down for color detection
                linkage1.setPosition(downpos);
                linkage2.setPosition(downpos);
                linkage3.setPosition(downpos);
                sleep(500); // Wait for servos to settle

                // Launch all 3 balls in sequence
                while (opModeIsActive() && partofpattern < 3) {
                    // Read color sensors multiple times and stabilize
                    NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
                    Color.colorToHSV(colors1.toColor(), hsvValues);
                    double distance1 = ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM);
                    String color1 = determineColor(hsvValues[0], hsvValues[1], hsvValues[2]);
                    patternindividuallive[0] = color1;

                    NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
                    Color.colorToHSV(colors2.toColor(), hsvValues);
                    double distance2 = ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM);
                    String color2 = determineColor(hsvValues[0], hsvValues[1], hsvValues[2]);
                    patternindividuallive[1] = color2;

                    NormalizedRGBA colors3 = colorSensor3.getNormalizedColors();
                    Color.colorToHSV(colors3.toColor(), hsvValues);
                    double distance3 = ((DistanceSensor) colorSensor3).getDistance(DistanceUnit.CM);
                    String color3 = determineColor(hsvValues[0], hsvValues[1], hsvValues[2]);
                    patternindividuallive[2] = color3;

                    // Display current status with detailed info
                    telemetry.addData("Pattern needed", pattern[patnum]);
                    telemetry.addData("Looking for", patternindividual[patnum][partofpattern]);
                    telemetry.addData("Position 1 (linkage1)", "%s (%.1f cm)", color1, distance1);
                    telemetry.addData("Position 2 (linkage2)", "%s (%.1f cm)", color2, distance2);
                    telemetry.addData("Position 3 (linkage3)", "%s (%.1f cm)", color3, distance3);
                    telemetry.addData("Part of pattern", "%d/3", partofpattern + 1);
                    telemetry.update();

                    double dist = 5.5;
                    if (distance1 < dist && patternindividuallive[0] == "O") {
                        linkage1.setPosition(0.76);
                        sleep(200);
                        linkage1.setPosition(downpos);
                        sleep(200);
                    }
                    if (distance2 < dist && patternindividuallive[1] == "O") {
                        linkage2.setPosition(0.76);
                        sleep(200);
                        linkage2.setPosition(downpos);
                        sleep(200);
                    }
                    if (distance3 < dist && patternindividuallive[2] == "O") {
                        linkage3.setPosition(0.76);
                        sleep(200);
                        linkage3.setPosition(downpos);
                        sleep(200);
                    }

                    // Check which position has the correct ball and launch it
                    sleep(1000);
                    if (patternindividuallive[0].equals(patternindividual[patnum][partofpattern])) {
                        telemetry.addData("Launching", "Position 1");
                        telemetry.update();
                        linkage1.setPosition(uppos);
                        sleep(sleep_time_auto);
                        linkage1.setPosition(downpos);
                        partofpattern++;
                        sleep(sleep_time_auto);
                    } else if (patternindividuallive[1].equals(patternindividual[patnum][partofpattern])) {
                        telemetry.addData("Launching", "Position 2");
                        telemetry.update();
                        linkage2.setPosition(uppos);
                        sleep(sleep_time_auto);
                        linkage2.setPosition(downpos);
                        partofpattern++;
                        sleep(sleep_time_auto);
                    } else if (patternindividuallive[2].equals(patternindividual[patnum][partofpattern])) {
                        telemetry.addData("Launching", "Position 3");
                        telemetry.update();
                        linkage3.setPosition(uppos);
                        sleep(sleep_time_auto);
                        linkage3.setPosition(downpos);
                        partofpattern++;
                        sleep(sleep_time_auto);
                    }

                    sleep(100); // Small delay between checks
                }

                // PHASE 5: Cleanup
                launcherMotorLeft.setPower(0);
                launcherMotorRight.setPower(0);

                telemetry.addData("Status", "Complete! All balls launched.");
                telemetry.update();
            } else {
                telemetry.addData("Status", "No AprilTag detected - autonomous stopped");
                telemetry.update();
            }

            strafeRight(100,432);

            // Keep running until stop is pressed
            while (opModeIsActive()) {
                sleep(50);
            }
        }

        // Cleanup
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set and enable the processor
        builder.addProcessor(aprilTag);

        // Build the Vision Portal
        visionPortal = builder.build();
    }

    /**
     * Determine color from HSV values
     */
    public static String determineColor(double h, double s, double v) {
        // Check if hue falls within the green range
        if (h >= 75 && h <= 185) {
            return "g";
        }
        // Check if hue falls within the purple range
        else if (h >= 210 && h <= 320) {
            return "p";
        }
        // If it's neither, return "Other"
        else {
            return "O";
        }
    }

    /**
     * Movement functions
     */

    // Stop all drive motors
    private void stopDriveMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    // Move forward
    public void moveForward(double power, long timeMs) {
        double axial = power;
        double lateral = 0;
        double yaw = 0;

        frontLeftDrive.setPower(axial + lateral + yaw);
        frontRightDrive.setPower((axial - lateral) - yaw);
        backLeftDrive.setPower((axial - lateral) + yaw);
        backRightDrive.setPower((axial + lateral) - yaw);

        sleep(timeMs);
        stopDriveMotors();
    }

    // Move backward
    public void moveBackward(double power, long timeMs) {
        double axial = -power;
        double lateral = 0;
        double yaw = 0;

        frontLeftDrive.setPower(axial + lateral + yaw);
        frontRightDrive.setPower((axial - lateral) - yaw);
        backLeftDrive.setPower((axial - lateral) + yaw);
        backRightDrive.setPower((axial + lateral) - yaw);

        sleep(timeMs);
        stopDriveMotors();
    }

    // Strafe left
    public void strafeLeft(double power, long timeMs) {
        double axial = 0;
        double lateral = -power;
        double yaw = 0;

        frontLeftDrive.setPower(axial + lateral + yaw);
        frontRightDrive.setPower((axial - lateral) - yaw);
        backLeftDrive.setPower((axial - lateral) + yaw);
        backRightDrive.setPower((axial + lateral) - yaw);

        sleep(timeMs);
        stopDriveMotors();
    }

    // Strafe right
    public void strafeRight(double power, long timeMs) {
        double axial = 0;
        double lateral = power;
        double yaw = 0;

        frontLeftDrive.setPower(axial + lateral + yaw);
        frontRightDrive.setPower((axial - lateral) - yaw);
        backLeftDrive.setPower((axial - lateral) + yaw);
        backRightDrive.setPower((axial + lateral) - yaw);

        sleep(timeMs);
        stopDriveMotors();
    }

    // Rotate clockwise
    public void rotateClockwise(double power, long timeMs) {
        double axial = 0;
        double lateral = 0;
        double yaw = power;

        frontLeftDrive.setPower(axial + lateral + yaw);
        frontRightDrive.setPower((axial - lateral) - yaw);
        backLeftDrive.setPower((axial - lateral) + yaw);
        backRightDrive.setPower((axial + lateral) - yaw);

        sleep(timeMs);
        stopDriveMotors();
    }

    // Rotate counter-clockwise
    public void rotateCounterClockwise(double power, long timeMs) {
        double axial = 0;
        double lateral = 0;
        double yaw = -power;

        frontLeftDrive.setPower(axial + lateral + yaw);
        frontRightDrive.setPower((axial - lateral) - yaw);
        backLeftDrive.setPower((axial - lateral) + yaw);
        backRightDrive.setPower((axial + lateral) - yaw);

        sleep(timeMs);
        stopDriveMotors();
    }
}
