/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "AutoV4 Blue BackStage", group = "Concept")
//@Disabled
public class AutoV4BlueClose extends LinearOpMode {

    double DESIRED_DISTANCE = 5; // how close the camera should get to the object (inches)

    final double SPEED_GAIN  =  0.03  ;   // Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   // Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   // Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;

    static final double COUNTS_PER_MOTOR_REV = 537.7; //Ticks per revolution
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    static final double WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For 96 mm diameter - If 140mm use 5.51181
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DISTANCE_BETWEEN_PIXEL_AND_FRONT = 0; //(inches)

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor arm;
    private CRServo claw;
    private DcMotor armBoost;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private int DESIRED_TAG_ID = 0;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private int myExposure;

    //arm variables
    static final int MIN_ARM_POSITION = 0;
    static final int MAX_ARM_POSITION = 705;

    static final int ARM_ANGLE_POSITION_FROM_MAX = 180; //546
    private static final String TFOD_MODEL_ASSET = "BlueCubeNEW.tflite";
    private static final String[] LABELS = {
            "BlueCube1",
    };

    BHI260IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;



    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;


    @Override
    public void runOpMode() {

        //Variables
        boolean targetFound = false; //Set to true when an AprilTag target is detected
        boolean pixelFound = false; //Set to true when pixel is found on spike mark
        String pixelLocation = ""; //Set to center, left, or right spike mark
        double drive = 0; //forward speed
        double strafe = 0; //Strafe speed
        double turn = 0; //turning speed
        int currentStep = 1;
        ElapsedTime runtime = new ElapsedTime();

        initTfod();
        //setManualExposure(14, 250);
        //Initalizes the motors
        frontLeft = hardwareMap.dcMotor.get("Motor3");
        backLeft = hardwareMap.dcMotor.get("Motor2");
        frontRight = hardwareMap.dcMotor.get("Motor0");
        backRight = hardwareMap.dcMotor.get("Motor1");
        claw = hardwareMap.get(CRServo.class, "claw");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armBoost = hardwareMap.get(DcMotor.class, "armBoost");

        //Reverses motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Resets the encoders?
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Starts the encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sets up the gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        IMU.Parameters myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(myIMUparameters);


        //Arm motor
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Claw closes
        claw.setPower(0.2);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        runtime.reset(); //starts the timer
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                targetFound = false;
                desiredTag = null;
                telemetryTfod();
                telemetry.update();
                //runtime.reset();
                //Step 1 - use Tensorflow to check for team prop on left and center spike
                if (currentStep == 1) {
                    if(runtime.milliseconds() < 4000){
                        List<Recognition> currentRecognitions = tfod.getRecognitions();
                        //go through list of recognitions and look for cube
                        for(Recognition recognition : currentRecognitions){
                            telemetryTfod();
                            if(recognition.getLabel() == "BlueCube1"){
                                if((returnXPositionOfCube() >= 0) && (returnXPositionOfCube() <= 300)){
                                    pixelLocation = "left";
                                    pixelFound = true;
                                    currentStep = 2;

                                } else{
                                    pixelLocation = "center";
                                    pixelFound = true;
                                    currentStep = 2;
                                }
                            }
                            else {
                                sleep(50);
                            }
                        }
                    }
                    else {
                        pixelLocation = "right";
                        pixelFound = true;
                        currentStep = 2;
                    }
                }
                //Step 2 - move forward
                if(currentStep == 2){
                    moveDistance(0.5,-6);
                    stopRobot();
                    if(pixelLocation == "left"){
                        currentStep = 3;
                    } else if(pixelLocation == "center"){
                        currentStep = 8;
                    } else { //if Right position
                        moveDistance(0.5,-12);
                        currentStep = 10;
                    }
                }

                //Step 3 - turn towards left spike
                if(currentStep == 3){
                    turn(16);
                    stopRobot();
                    currentStep = 4;
                }
                //Step 4 - move forward towards left spike mark
                if(currentStep == 4){
                    moveDistance(0.5, -(14.5 + DISTANCE_BETWEEN_PIXEL_AND_FRONT));
                    stopRobot();
                    sleep(50);
                    currentStep = 5;
                }

                //Step 5 - back away from spike mark to drop off purple pixel
                if(currentStep == 5){
                    moveDistance(0.5, (14.5 + DISTANCE_BETWEEN_PIXEL_AND_FRONT));
                    stopRobot();
                    sleep(50);
                    currentStep = 6;
                }

                //Step 6 - point towards Center from left spike mark
                if(currentStep == 6){
                    turn(-(16));
                    stopRobot();
                    currentStep = 14;
                }

                //Step 8 - move forward towards center spike mark
                if(currentStep == 8){
                    moveDistance(0.5,-(20 + DISTANCE_BETWEEN_PIXEL_AND_FRONT));
                    stopRobot();
                    sleep(50);
                    currentStep = 9;
                }

                //Step 9 - back away from center mark, dropping off purple pixel
                if(currentStep == 9){
                    moveDistance(0.5,(20+DISTANCE_BETWEEN_PIXEL_AND_FRONT));
                    stopRobot();
                    currentStep = 14;
                }
                //Step 10 - turn towards right spike mark
                if(currentStep == 10){
                    turn(-(45));
                    stopRobot();
                    currentStep = 11;
                }

                //Step 11 - move forward towards right spike mark
                if(currentStep == 11){
                    moveDistance(0.5, -(10 + DISTANCE_BETWEEN_PIXEL_AND_FRONT));
                    stopRobot();
                    sleep(50);
                    currentStep = 12;
                }

                //Step 12 - move backward from right spike
                if(currentStep == 12){
                    moveDistance(0.5, (10 + DISTANCE_BETWEEN_PIXEL_AND_FRONT));
                    stopRobot();
                    currentStep = 13;
                }

                //Step 13 - point towards center from right spike
                if(currentStep == 13){
                    turn((45));
                    stopRobot();
                    currentStep = 14;
                }

                //Step 14 - turn to face backdrop
                if(currentStep == 14){
                    strafe(0.7,650,false);
                    stopRobot();
                    turn(90);
                    stopRobot();
                    currentStep = 18;
                }

                //STep 18 - strafe right to center around backdrop (or can put it to a certain amount depending on pixel location)
                if(currentStep == 18){
                    if(pixelLocation == "left"){
                        strafe(0.7, 200, true);
                    } else if (pixelLocation == "center"){
                        strafe(0.7,600, true);
                    } else{
                        strafe(0.7,710,true);
                    }
                    //stopRobot();
                    //moveDistance(0.5, -8);
                    stopRobot();
                    currentStep = 20;
                }
                //Step 20 - set up april tags
                if(currentStep == 20) {
                    visionPortal.close();
                    sleep(50);
                    //Start up april tag
                    initAprilTag();
                    sleep(50);
                    setManualExposure(4, 250); //reduce motion blur

                    if (pixelLocation == "left") {
                        DESIRED_TAG_ID = 1;
                    } else if (pixelLocation == "center") {
                        DESIRED_TAG_ID = 2;
                    } else {
                        DESIRED_TAG_ID = 3;
                    }

                    currentStep = 21;
                }
                //Step 21 - adjustment step
                if(currentStep == 21){
                    currentStep = 22;
                }
                //Step 22 - move to backdrop using april tag
                if(currentStep == 22){
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections){
                        if((detection.metadata != null) && (detection.id == DESIRED_TAG_ID)){
                            targetFound = true;
                            desiredTag = detection;
                            break; //dont look any futher
                        }
                    }

                    if(targetFound){
                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                        double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double  headingError    = desiredTag.ftcPose.bearing;
                        double  yawError        = desiredTag.ftcPose.yaw;

                        if ((rangeError < 7) && (Math.abs(headingError) < 5) && (Math.abs(yawError) < 5)){
                            //If close stop
                            drive = 0;
                            turn = 0;
                            strafe = 0;
                            currentStep = 23;
                        }
                        else{
                            // Use the speed and turn "gains" to calculate how we want the robot to move.
                            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                        }

                        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                        // Apply desired axes motions to the robot
                        telemetry.addData("Is it working: ", "Yes");
                        moveRobot(-drive, -strafe, turn);
                        sleep(10);
                        stopRobot();
                    }
                    else {
                        sleep(10);
                        stopRobot();
                    }
                }

                //Step 23 - May need an adjust step
                if(currentStep == 23){
                    strafe(0.5,300,false);
                    currentStep = 24;
                    runtime.reset();
                }
                //Step 24 - raise arm to drop position
                if (currentStep == 24){
                    arm.setPower(0.55);
                    ((DcMotorEx) arm).setVelocity(1000);
                    arm.setTargetPosition(MAX_ARM_POSITION - ARM_ANGLE_POSITION_FROM_MAX);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(arm.isBusy()){

                    }
                    sleep(300);
                    claw.setPower(-1); //drops pixel
                    sleep(200);
                    currentStep = 25;
                }
                //Step 25 - lower arm
                if(currentStep == 25){
                    arm.setTargetPosition(MIN_ARM_POSITION);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(100);
                    currentStep = 26;
                    runtime.reset();
                }
                //Step 26 - move towards wall and finish
                if(currentStep == 26){
                    if(pixelLocation == "left")
                    {
                        strafe(0.7,300,false);
                    }
                    else if (pixelLocation == "center"){
                        strafe(0.7,600,false);
                    } else{
                        strafe(0.7,900,false);
                    }
                    sleep(10);
                    claw.setPower(0);
                    turn(90);
                    if(pixelLocation != "left"){
                        moveDistance(0.5,-6);
                    }
                    stopRobot();
                    currentStep = 27;
                }

                //backup step because idk how to fix april tag
                if(currentStep == 28){
                    moveDistance(0.5, -15);
                    stopRobot();
                    arm.setPower(0.55);
                    ((DcMotorEx) arm).setVelocity(1000);
                    arm.setTargetPosition(MAX_ARM_POSITION - ARM_ANGLE_POSITION_FROM_MAX);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(arm.isBusy()){

                    }
                    sleep(300);
                    claw.setPower(-1); //drops pixel
                    sleep(200);

                    arm.setTargetPosition(MIN_ARM_POSITION);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(100);
                    //currentStep = 26;

                    currentStep = 29;
                }

                //Step 27 - may need to rotate to a good position
                // Push telemetry to the Driver Station.
                telemetry.addData("current step", currentStep);
                telemetry.addData("pixel found", pixelFound);
                telemetry.addData("pixel location", "%s", pixelLocation);
                telemetry.addData("tag target", DESIRED_TAG_ID);
                telemetry.addData("tag found", targetFound);
                telemetry.addData("x Position", returnXPositionOfCube());
                telemetry.update();

            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.80f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
    private double returnXPositionOfCube(){
        double xPos = -1;
        List<Recognition> currentReconitions = tfod.getRecognitions();
        for(Recognition recognition : currentReconitions){
            xPos = (recognition.getLeft() + recognition.getRight()) / 2;
            break;
        }
        return xPos;
    }

    private void moveRobot(double x, double y, double yaw){
        double frontLeftPower = x-y-yaw;
        double frontRightPower = x+y+yaw;
        double backLeftPower = x+y-yaw;
        double backRightPower = x-y+yaw;

        //powers less than 1
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        //Sets power to teh wheels
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }
    private void stopRobot(){
        setAllPower(0);
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
    private void setManualExposure(int exposureMS, int gain){
        if(visionPortal == null){
            return;
        }

        if(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while(!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)){
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        //Honestly I have no idea what this does
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    public void moveDistance(double power, double distance){
        //Resets the encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the target position to the distance
        frontLeft.setTargetPosition((int)(distance * (int) COUNTS_PER_INCH));
        backLeft.setTargetPosition((int)(distance * (int) COUNTS_PER_INCH));
        frontRight.setTargetPosition((int)(distance * (int) COUNTS_PER_INCH));
        backRight.setTargetPosition((int)(distance * (int) COUNTS_PER_INCH));

        //Takes motors to that position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Goes forward at the certain speed
        setAllPower(power);

        //Waits until the motors are done moving
        while(frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy()){

        }

        //Stops the motors
        stopRobot();

        //Goes back to running using the encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setAllPower(double power) {
        setMotorPower(power, power, power, power);
    }
    public void setMotorPower(double frontL, double backL, double frontR, double backR){
        frontLeft.setPower(frontL);
        backLeft.setPower(backL);
        frontRight.setPower(frontR);
        backRight.setPower(backR);
    }
    //Strafes Left for false, right for true
    public void strafe(double power, long time, boolean direction){
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Goes forward at the certain speed
        if(direction){
            setMotorPower(-power,power,power,-power);
        }
        else{
            setMotorPower(power,-power,-power,power);
        }
        sleep(time);
        //Stops the motors
        stopRobot();

        //Goes back to running using the encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetAngle(){
        //Need to change the axes order based on orientation of the extension hub
        //Look up IMU interface
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle(){
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        //delta Angle is only between -179 and 180
        if (deltaAngle > 180){
            deltaAngle -= 360;
        } else if (deltaAngle <= -180){
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        telemetry.update();
        return currAngle;
    }
    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while(opModeIsActive() && Math.abs(error) > 2){
            //Sets power to 0.3 depending on if the error is negative or positive(right or left)
            double motorPower = (error < 0 ? -0.3 : 0.3);
            //Left is negative right is positive
            setMotorPower(-motorPower, -motorPower, motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.addData("gyro", getAngle());
            telemetry.update();

        }

        stopRobot();
    }
    public void turnTo(double degrees){
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if(error > 180){
            error -= 360;
        } else if (error < -180){
            error += 360;
        }

        turn(error);
    }
}   // end class
