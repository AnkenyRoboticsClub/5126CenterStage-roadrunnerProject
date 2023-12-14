package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "AutoV5BlueBackstage",group="Concpet")
public class AutoV5BlueBackstage extends LinearOpMode {
    private DcMotor arm;
    private CRServo claw;
    private DcMotor armBoost;
    private int armDropPosition = 523;

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
        boolean blockFound = false; //Set to true when pixel is found on spike mark
        String blockLocation = ""; //Set to center, left, or right spike mark
        int currentStep = 1;
        ElapsedTime runtime = new ElapsedTime();

        initTfod();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        claw = hardwareMap.get(CRServo.class, "claw");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armBoost = hardwareMap.get(DcMotor.class, "armBoost");

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
        runtime.reset();
        if(opModeIsActive()){
            while(opModeIsActive()){
                blockFound = false;
                telemetryTfod();
                telemetry.update();
                drive.pose = new Pose2d(14.08, 62.61, Math.toRadians(-90.00));
                //runtime.reset();

                //Step 1 - use Tensorflow to check for team prop on left and center spike
                if (currentStep == 1) {
                    if(runtime.milliseconds() < 4000){
                        List<Recognition> currentRecognitions = tfod.getRecognitions();
                        //go through list of recognitions and look for cube
                        for(Recognition recognition : currentRecognitions){
                            telemetryTfod();
                            if(recognition.getLabel() == "BlueCube1"){
                                //Detects the position of the cube on the screen - should be the same as RedFrontStage
                                if((returnXPositionOfCube() >= 0) && (returnXPositionOfCube() <= 400)){
                                    //Location Left
                                    //blockLocation = "left";
                                    //blockFound = true;
                                    currentStep = 2;

                                } else{
                                    //Location Center
                                    //blockLocation = "center";
                                    //blockFound = true;
                                    currentStep = 3;
                                }
                            }
                            else {
                                sleep(50);
                            }
                        }
                    }
                    else {
                        //Location Right
                        //blockLocation = "right";
                        //blockFound = true;
                        currentStep = 4;
                    }
                }

                //Step 2 - Left line Start
                if(currentStep == 2){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(18,36.5),Math.toRadians(-45))
                                    .setReversed(true)
                                    .splineToConstantHeading(new Vector2d(18.56,55), Math.toRadians(-90))
                                    .setReversed(false)
                                    .splineTo(new Vector2d(46.5, 44.85), Math.toRadians(0))
                                    .build()
                    );
                    currentStep = 10;
                }

                //Step 3 - Middle line Start
                if(currentStep == 3){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(12, 32), Math.toRadians(-90))
                                    .setReversed(true)
                                    .splineToConstantHeading(new Vector2d(15, 50), Math.toRadians(-90))
                                    .setReversed(false)
                                    .splineTo(new Vector2d(46.5, 38), Math.toRadians(0.00))
                                    .build()
                    );
                    currentStep = 10;
                }

                //Step 4 - Right line Start
                if(currentStep == 4){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(7, 37), Math.toRadians(-135))
                                    .setReversed(true)
                                    .splineToConstantHeading(new Vector2d(23, 53), Math.toRadians(-90))
                                    .setReversed(false)
                                    .splineTo(new Vector2d(46.72, 29.68), Math.toRadians(0.00))
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
                    if(armDropPosition + 5 < arm.getCurrentPosition() && armDropPosition - 5 < arm.getCurrentPosition()){
                        claw.setPower(-1);
                        currentStep = 11;
                    }

                }
                //Step 11 - Lower arm
                if(currentStep == 11){
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(100);
                    currentStep = 12;
                }

                //Step 12 - Parking
                if(currentStep == 12){
                    //drive.pose = new Pose2d(46.72, -29.68, Math.toRadians(0));
                    drive.updatePoseEstimate();
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .lineToYConstantHeading(60)
                                    .build()
                    );
                }
            }
        }
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
