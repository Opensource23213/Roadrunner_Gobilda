

package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.sensors;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * FFTCOpenSourceAutonomouss Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "FTCOpenSourceAutonomouswAprilTagsl", group = "00-Autonomous", preselectTeleOp = "TyneyCompetitionCode")
public class FTCOpenSourceAutonomouswApriTagsl extends LinearOpMode {

    public static String TEAM_NAME = "Open Source Robotics";
    public static int TEAM_NUMBER = 23213;
    private static final String TFOD_MODEL_FILE = "myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue Prop", "Red Prop"
    };
    private AprilTagProcessor aprilTag;
    private CRServo spinny;

    public SampleMecanumDrive drive;
    public sensors sense;
    private DcMotor Arm1 = null;
    private DcMotor Arm2 = null;
    private Servo elbow1;
    private Servo wristy;
    private Servo grabby;
    private Servo flippy;
    private DcMotor ArmPos = null;
    private Servo mustaches;
    private Servo ilifty;
    private DistanceSensor sensorDistance;
    private DistanceSensor sensorDistance2;
    private DistanceSensor sensorDistancer;
    private DistanceSensor sensorDistancel;
    private DistanceSensor sensorDistanceb;
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor rear_left = null;
    private DcMotor rear_right = null;
    int use_tfod = 1;
    int use_apriltag = 2;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private VisionPortal myVisionPortal;

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_NEAR,
        BLUE_FAR,
        RED_FAR,
        RED_NEAR
    }

    public static START_POSITION startPosition;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);  //Moved from void automation
        sensors sense = new sensors(hardwareMap);                        //Moved from void automation
        spinny = hardwareMap.get(CRServo.class, "spinny");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        elbow1 = hardwareMap.get(Servo.class, "elbow1");
        grabby = hardwareMap.get(Servo.class, "grabby");
        wristy = hardwareMap.get(Servo.class, "wristy");
        ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
        flippy = hardwareMap.get(Servo.class, "flippy");
        mustaches = hardwareMap.get(Servo.class, "mustaches");
        ilifty = hardwareMap.get(Servo.class, "lifty2");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensorDistance2");
        sensorDistancel = hardwareMap.get(DistanceSensor.class, "dis_left");
        sensorDistancer = hardwareMap.get(DistanceSensor.class, "dis_right");
        sensorDistanceb = hardwareMap.get(DistanceSensor.class, "dis_rear");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) sensorDistance2;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d poseEstimate = drive.getPoseEstimate();
        Arm1.setDirection(DcMotor.Direction.FORWARD);
        Arm2.setDirection(DcMotor.Direction.REVERSE);
        ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow1.setDirection(Servo.Direction.REVERSE);
        ilifty.setPosition(.3);

        //Key Pay inputs to selecting Starting Position of robot
        //grabby.setPosition(.77);
        //mustaches.setPosition(.28);
        //Activate Camera Vision that uses TensorFlow for pixel detection
        initDoubleVision();
        myVisionPortal.setProcessorEnabled(aprilTag, false);
        myVisionPortal.setProcessorEnabled(tfod, true);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData("|>", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            myVisionPortal.setProcessorEnabled(aprilTag, true);
            myVisionPortal.setProcessorEnabled(tfod, false);
            telemetryAprilTag();
            myVisionPortal.setProcessorEnabled(aprilTag, false);
            myVisionPortal.setProcessorEnabled(tfod, true);
            runTfodTensorFlow();
            //identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;  //REMOVE!!!!

            telemetry.addData("Vision identified Parking Location", identifiedSpikeMarkLocation);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            myVisionPortal.setProcessorEnabled(aprilTag, true);
            myVisionPortal.setProcessorEnabled(tfod, false);
            runAutonoumousMode();
        }
    }   // end runOpMode()

    private void stack1() {
        spinny.setPower(1);
        flippy.setPosition(.475);
        mustaches.setPosition(.42);
        safeWaitSeconds(.6);
    }

    public void stack2() {
        mustaches.setPosition(.68);
        spinny.setPower(0);
    }

    public void in() {
        spinny.setPower(1);
        mustaches.setPosition(.28);
    }

    public void stopp() {
        spinny.setPower(0);
        flippy.setPosition(.23);
    }

    public void release() {
        ilifty.setPosition(.3);
        mustaches.setPosition(.68);
        safeWaitSeconds(.1);
    }

    public void b() {
        elbow1.setPosition(.91);
        wristy.setPosition((float) .6 + 3 / 355);
        armpose(0);
    }

    public Pose2d getaprilpose(int id, int x, int y, int angle) {
        float xpose = 0;
        float ypose = 0;
        AprilTagDetection desiredTag = null;
        while(xpose == 0) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((detection.id == id)) {
                        // Yes, we want to use this tag.
                        desiredTag = detection;
                        telemetry.addData("dis", String.valueOf(desiredTag.ftcPose.y), desiredTag.ftcPose.x);
                        telemetry.update();
                        xpose = (int) desiredTag.ftcPose.x;
                        ypose = (int) desiredTag.ftcPose.y;
                        xpose += 8;
                        ypose -= 4.5;
                        x += xpose;
                        y += ypose;
                    } else {

                    }
                }

            }
        }
        return new Pose2d(x, y, Math.toRadians(angle));
    }
    public void pick(){
        double grabIn = 1;
        grabby.setPosition(grabIn);
        wristy.setPosition((float) ((float) .65 + 3/355));
        safeWaitSeconds(.25);//
        elbow1.setPosition(1);
        safeWaitSeconds(.5);//
        grabby.setPosition(.75);
        safeWaitSeconds(.2);//
        flippy.setPosition(.5);
        elbow1.setPosition(.91);
    }
    public double dis(String side) {
        double dis = 1;
        if (side == "front") {
            dis = (sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH)) / 2;
        }
        if (side == "frontr") {
            dis = sensorDistance2.getDistance(DistanceUnit.INCH);
        }
        if (side == "frontl") {
            dis = sensorDistance.getDistance(DistanceUnit.INCH);
        }
        if (side == "right") {
            dis = sensorDistancer.getDistance(DistanceUnit.INCH);;
        }
        if (side == "left") {
            dis = sensorDistancel.getDistance(DistanceUnit.INCH);;
        }
        if (side == "back") {
            dis = sensorDistanceb.getDistance(DistanceUnit.INCH);;
        }
        return dis;
    }

    public double distancechange(float num, String side){
        double dis = dis(side);
        dis = dis - num;
        return dis;
    }
    public double distancechanges(float num, String side){
        double dis = dis(side);
        if (dis < 10){
             dis = 0;
        }
        dis = dis - num;
        return dis;
    }
    public void drop(){
        wristy.setPosition(0);
        grabby.setPosition(1);
        double grab = 1;
        double dis = sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH);
        dis /= 2;
        double dis2 = dis + 1;
        while (grab == 1){
            if (dis2 > dis) {
                dis = (sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH))/2;
                front_left.setPower(-.2);
                front_right.setPower(-.2);
                rear_left.setPower(-.2);
                rear_right.setPower(-.2);
            } else{
                front_left.setPower(0);
                front_right.setPower(0);
                rear_left.setPower(0);
                rear_right.setPower(0);
                grab = 2;
            }
        }
        safeWaitSeconds(1);
        /*grabby.setPosition(.75);
        armpose(0);
        dis2 = dis - .6;
        while (grab == 2){
            if (dis > dis2) {
                dis = (sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH))/2;
                front_left.setPower(.2);
                front_right.setPower(.2);
                rear_left.setPower(.2);
                rear_right.setPower(.2);
            } else{
                front_left.setPower(0);
                front_right.setPower(0);
                rear_left.setPower(0);
                rear_right.setPower(0);
                grab = 3;
            }
        }
        grabby.setPosition(1);
        wristy.setPosition(0);
        safeWaitSeconds(.5);*/
        b();
        safeWaitSeconds(10);
    }

    public void armout(){
        armpose(-4);
        elbow1.setPosition(0);
        wristy.setPosition(.0293);
    }
    public double distancechange(int now, int later, String sensor){
        if(now - 7 - 8.375 > dis(sensor)){
            later -= (int) (now - (dis(sensor) + 8.375));
            later = (int) distancechange(later, sensor);
        }else{
            later = (int) distancechange(later, sensor);
        }

        return later;
    }
    public void armpose(int pose){
        double ticks = 22.76;
        double armAngle = ArmPos.getCurrentPosition() / ticks - 25;
        while (armAngle != pose) {
            armAngle = ArmPos.getCurrentPosition() / ticks - 25;
            if (pose < armAngle + 1 && pose > armAngle - 1) {// Stop arm movement within a 4 degree range
                Arm1.setPower(0);
                Arm2.setPower(0);
                break;

            } else if (pose > armAngle + 8 || pose < armAngle - 8) {//  Far and fast arm move into position within an infinite range
                if (pose < armAngle) {
                    Arm1.setPower(1);
                    Arm2.setPower(1);
                }
                if (pose > armAngle) {
                    Arm1.setPower(-1);
                    Arm2.setPower(-1);
                }

            } else { //Close and slow arm move into position if arm is in a 16 degree range
                if (pose < armAngle) {
                    Arm1.setPower(.2);
                    Arm2.setPower(.2);
                }
                if (pose > armAngle) {
                    Arm1.setPower(-.2);
                    Arm2.setPower(-.2);
                }

            }
        }
    }
    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);  //Moved with other hardware mapping

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(1)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        double ticks = 22.76;
        double armAngle = ArmPos.getCurrentPosition() / ticks - 25;
        double serAdjust = -7;
        //elbow1.setPosition(.91);
        double grabIn = 1;
        float wristIn = (float) ((float) .6 + serAdjust/355);
        //wristy.setPosition(wristIn);
        double grabOut = .79;
        float servoDegree = 1 / 355;
        telemetry.update();
        armpose(0);
        grabby.setPosition(.77);
        elbow1.setPosition(0);
        wristy.setPosition(0);
        //Move robot to dropPurplePixel based on identified Spike Mark Location
        while(opModeIsActive()) {
            traj3= drive.trajectorySequenceBuilder(new Pose2d())
                    .turn(Math.toRadians(90))
                    .build();
            drive.followTrajectorySequence(traj3);
            traj1 = drive.trajectorySequenceBuilder(traj3.end())
                    .lineToLinearHeading(getaprilpose(1,0,0,90))
                    .build();
            drive.followTrajectorySequence(traj1);
            break;
        }


        //TODO : Code to drop Pixel on Backdrop
        /*elbow1.setPosition(0);
        wristy.setPosition(13/355);
        safeWaitSeconds(1 );
        grabby.setPosition(1);
        safeWaitSeconds(.4 );
*/
    }



    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FFTCOpenSourceAutonomousourceAutonomousnomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Near   ", "(X / ▢)");
            telemetry.addData("    Blue Far ", "(Y / Δ)");
            telemetry.addData("    Red Far    ", "(B / O)");
            telemetry.addData("    Red Near  ", "(A / X)");

            if(gamepad1.x && gamepad2.x){
                startPosition = START_POSITION.BLUE_NEAR;
                break;
            }
            if(gamepad1.y && gamepad2.y){
                startPosition = START_POSITION.BLUE_FAR;
                break;
            }
            if(gamepad1.b && gamepad2.b){
                startPosition = START_POSITION.RED_FAR;
                break;
            }
            if(gamepad1.a && gamepad2.a){
                startPosition = START_POSITION.RED_NEAR;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();

    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */

    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)

                .build();
        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
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
                telemetry.addLine(String.format("", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    private void setCamMode() {
        if (use_apriltag == 1) {
            myVisionPortal.setProcessorEnabled(aprilTag, true);
        } else {
            myVisionPortal.setProcessorEnabled(aprilTag, false);
        }
        if (use_tfod == 1) {
            myVisionPortal.setProcessorEnabled(tfod, true);
        } else {
            myVisionPortal.setProcessorEnabled(tfod, false);
        }
    }   // end method doCameraSwitching()

// end class
    private void runTfodTensorFlow() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        //Camera placed between Left and Right Spike Mark on RED_FAR and BLUE_NEAR If pixel not visible, assume Right spike Mark
        if (startPosition == FTCOpenSourceAutonomouswApriTagsl.START_POSITION.RED_FAR || startPosition == FTCOpenSourceAutonomouswApriTagsl.START_POSITION.BLUE_NEAR) {
            identifiedSpikeMarkLocation = FTCOpenSourceAutonomouswApriTagsl.IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
        } else { //RED_NEAR or BLUE_FAR
            identifiedSpikeMarkLocation = FTCOpenSourceAutonomouswApriTagsl.IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
        }
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if (recognition.getLabel() == "Blue Prop" || recognition.getLabel() == "Red Prop") {
                if (x < 350) {
                    identifiedSpikeMarkLocation = FTCOpenSourceAutonomouswApriTagsl.IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                } else {
                    identifiedSpikeMarkLocation = FTCOpenSourceAutonomouswApriTagsl.IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                }
            } else {
                identifiedSpikeMarkLocation = FTCOpenSourceAutonomouswApriTagsl.IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
            }

        }// end for() loop

    }   // end method runTfodTensorFlow()
}   // end class
