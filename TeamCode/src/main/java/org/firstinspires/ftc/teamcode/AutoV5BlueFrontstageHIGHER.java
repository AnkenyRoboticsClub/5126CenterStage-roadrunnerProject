package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutoV5BlueFrontstage",group="Concpet")
public class AutoV5BlueFrontstageHIGHER extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor arm;
    private CRServo claw;
    private DcMotor armBoost;
    private int armDropPosition = 542;
    static final double COUNTS_PER_MOTOR_REV = 537.7; //Ticks per revolution
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    static final double WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For 96 mm diameter - If 140mm use 5.51181
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private VisionPortal visionPortal;
    private TfodProcessor tfod;
    private int myExposure;

    private static final String TFOD_MODEL_ASSET = "BlueCubeNEW.tflite";
    private static final String[] LABELS = {
            "BlueCube1",
    };

    @Override
    public void runOpMode(){
        String blockLocation = ""; //Set to Left, Center, or Right depending on where the camera detects the block
        int currentStep = 1;
        ElapsedTime runtime = new ElapsedTime();

        initTfod();
        MecanumDriveSLOW drive = new MecanumDriveSLOW(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        drive.defaultAccelConstraint.equals(30);
        drive.defaultVelConstraint.equals(30);

        claw = hardwareMap.get(CRServo.class, "claw");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armBoost = hardwareMap.get(DcMotor.class, "armBoost");

        //Initalizes motors for strafing
        frontLeft = hardwareMap.get(DcMotorEx.class, "Motor1");
        backLeft = hardwareMap.get(DcMotorEx.class, "Motor0");
        backRight = hardwareMap.get(DcMotorEx.class, "Motor3");
        frontRight = hardwareMap.get(DcMotorEx.class, "Motor2");

        //Arm motor
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Claw closes
        claw.setPower(0.1);


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();


        waitForStart();
        runtime.reset();
        if(opModeIsActive()){
            while(opModeIsActive()){
                telemetryTfod();
                telemetry.update();

                drive.pose = new Pose2d(-37.77, 62.9, Math.toRadians(-90.00));

                //Step 1 - use Tensorflow to check for team prop on left and center spike
                if (currentStep == 1) {
                    if(runtime.milliseconds() < 1500){
                        List<Recognition> currentRecognitions = tfod.getRecognitions();
                        //go through list of recognitions and look for cube
                        for(Recognition recognition : currentRecognitions){
                            telemetryTfod();
                            if(recognition.getLabel() == "BlueCube1"){
                                if((returnXPositionOfCube() >= 0) && (returnXPositionOfCube() <= 250)){
                                    // Location Left
                                    blockLocation = "left";
                                    currentStep = 2;

                                } else{
                                    // Location Center
                                    blockLocation = "center";
                                    currentStep = 3;
                                }
                            }
                            else {
                                sleep(50);
                            }
                        }
                    }
                    else {
                        // Location Right
                        blockLocation = "right";
                        currentStep = 4;
                    }
                }
                //Step 2 - Left line Start
                if(currentStep == 2){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(-32, 38.06), Math.toRadians(-45.00))
                                    .setReversed(true)
                                    .splineToConstantHeading(new Vector2d(-45, 54.09), Math.toRadians(-45))
                                    .setReversed(false)
                                    .splineTo(new Vector2d(-30, 12), Math.toRadians(0))
                                    .splineTo(new Vector2d(-18.89, 12), Math.toRadians(0))
                                    .splineTo(new Vector2d(27.89, 12), Math.toRadians(0))
                                    .splineToConstantHeading(new Vector2d(43.75, 44.85), Math.toRadians(0.00))
                                    .build()
                    );
                    currentStep = 10;
                }

                //Step 3 - Center line Start
                if(currentStep == 3){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(-35.60, 36), Math.toRadians(-90.00))
                                    .setReversed(true)
                                    .splineToConstantHeading(new Vector2d(-36.55, 53.07), Math.toRadians(-90))
                                    .setReversed(false)
                                    .splineToConstantHeading(new Vector2d(-50, 50), Math.toRadians(-90))
                                    .splineTo(new Vector2d(-40.98, 12), Math.toRadians(0.00))
                                    .splineTo(new Vector2d(31.44, 12), Math.toRadians(0))
                                    .splineToConstantHeading(new Vector2d(43.75, 38), Math.toRadians(0.00))
                                    .build()
                    );

                    currentStep = 10;
                }


                //Step 4 - Right line Start
                if(currentStep == 4){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(-42, 39.94), Math.toRadians(-119.00))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-35, 59.29), Math.toRadians(-267.51))
                                    .setReversed(false)
                                    .splineTo(new Vector2d(-20, 13), Math.toRadians(0))
                                    .splineTo(new Vector2d(29.97, 13), Math.toRadians(0))
                                    .splineToConstantHeading(new Vector2d(43.75, 29.68), Math.toRadians(0.00))
                                    .build()
                    );
                    currentStep = 10;
                }

                //Step 10-11 Arm up release arm down
                //Step 10 - Raise arm
                if (currentStep == 10){
                    arm.setTargetPosition(armDropPosition);
                    arm.setPower(0.5);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(4000);
                    //Wait until arm is within the bounds of the set position before moving onto next step
                    /*
                    if(armDropPosition + 5 < arm.getCurrentPosition() && armDropPosition - 5 < arm.getCurrentPosition()){
                        claw.setPower(-1);
                        currentStep = 11;
                    }

                     */
                    claw.setPower(-1);
                    currentStep = 11;
                }
                //Step 11 - Lower arm
                if(currentStep == 11){
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //sleep because sometimes it skips loop
                    sleep(2000);
                    //Waits until the arm stops moving before going forward
                    while(arm.isBusy()){

                    }
                    currentStep = 12;
                }

                //Step 12 - Turning
                if(currentStep == 12){
                    drive.updatePoseEstimate();

                    //Turns -90 degrees back to starting position
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turn(Math.toRadians(-90))
                                    .build()
                    );
                    currentStep = 13;
                }

                //Step 13 - parking
                if(currentStep == 16){
                    if(blockLocation == "right"){
                        moveDistance(0.7,11);
                    }else if (blockLocation == "center"){
                        moveDistance(0.7,16);
                    }else if(blockLocation == "left"){
                        moveDistance(0.7,22);
                    } else{
                        break;
                    }
                    currentStep = 14;
                }
            }
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
        setMotorPower(power, power, power, power);

        //Waits until the motors are done moving
        while(frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy()){

        }

        //Stops the motors
        setMotorPower(0,0,0,0);

        //Goes back to running using the encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setMotorPower(double frontL, double backL, double frontR, double backR){
        frontLeft.setPower(frontL);
        backLeft.setPower(backL);
        frontRight.setPower(frontR);
        backRight.setPower(backR);
    }
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
}
