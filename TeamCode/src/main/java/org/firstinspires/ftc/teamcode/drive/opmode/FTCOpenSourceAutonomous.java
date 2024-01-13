

package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.sensors;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * FFTCOpenSourceAutonomouss Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "FTCOpenSourceAutonomous", group = "00-Autonomous", preselectTeleOp = "TyneyCompetitionCode")
public class FTCOpenSourceAutonomous extends LinearOpMode {

    public static String TEAM_NAME = "Open Source Robotics";
    public static int TEAM_NUMBER = 23213;
    private static final String TFOD_MODEL_FILE = "myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue Prop", "Red Prop"
    };

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

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
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
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        elbow1 = hardwareMap.get(Servo.class, "elbow1");
        grabby = hardwareMap.get(Servo.class, "grabby");
        wristy = hardwareMap.get(Servo.class, "wristy");
        ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
        flippy = hardwareMap.get(Servo.class, "flippy");
        mustaches = hardwareMap.get(Servo.class, "mustaches");
        ilifty = hardwareMap.get(Servo.class, "lifty2");
        Arm1.setDirection(DcMotor.Direction.FORWARD);
        Arm2.setDirection(DcMotor.Direction.REVERSE);
        ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow1.setDirection(Servo.Direction.REVERSE);
        ilifty.setPosition(.3);

        //Key Pay inputs to selecting Starting Position of robot
        grabby.setPosition(.79);
        mustaches.setPosition(.28);
        selectStartingPosition();
        //Activate Camera Vision that uses TensorFlow for pixel detection
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData("|>", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            //runTfodTensorFlow();
            identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;  //REMOVE!!!!

            telemetry.addData("Vision identified Parking Location", identifiedSpikeMarkLocation);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()
    private void stack1() {
        spinny.setPower(1);
        flippy.setPosition(.475);
        mustaches.setPosition(.42);
        safeWaitSeconds(.5);
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
        flippy.setPosition(.13);
    }
    public void release() {
        ilifty.setPosition(.3);
        mustaches.setPosition(.68);
        safeWaitSeconds(.4);
    }
    public void pick(){
        double xpress = 2;
        double counter3 = 0;
        double pickflip = .13 ;
        double grabIn = 1;
        double grabOut = .85;
        double flippydoo = pickflip;
        float wristIn = (float) ((float) .6 + 4/355);
        while (xpress != 1) {
            if (xpress == 2 && counter3 < 2) {
                counter3 += 1;
            } else if (xpress == 2 && counter3 >= 2) {
                flippy.setPosition(flippydoo);
                xpress = 3;
                counter3 = 0;
            } else if (xpress == 3 && counter3 < 12) {
                counter3 += 1;
            } else if (xpress == 3 && counter3 >= 12) {
                elbow1.setPosition(1);
                xpress = 4;
                counter3 = 0;
            } else if (xpress == 4 && counter3 < 4) {
                counter3 += 1;
            } else if (xpress == 4 && counter3 >= 4) {
                grabby.setPosition(grabOut);
                xpress = 5;
                counter3 = 0;
            } else if (xpress == 5 && counter3 < 7) {
                counter3 += 1;
            } else if (xpress == 5 && counter3 >= 7) {
                flippydoo = .5;
                flippy.setPosition(flippydoo);
                xpress = 6;
                counter3 = 0;
            } else if (xpress == 6 && counter3 < 12) {
                counter3 += 1;
            } else if (xpress == 6 && counter3 >= 12) {
                elbow1.setPosition(.91);
                xpress = 1;
                counter3 = 0;
                wristy.setPosition(wristIn);
            }
        }
    }
    public void armout(){
        elbow1.setPosition(0);
        wristy.setPosition((20 + 3 + -5 * 1.4 - 4 * 1.4) / 355);
        safeWaitSeconds(.4);
    }
    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);  //Moved with other hardware mapping
        sensors sense = new sensors(hardwareMap);

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

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        while(opModeIsActive()) {
            double ticks = 22.76;
            double armAngle = ArmPos.getCurrentPosition() / ticks - 22.5;
            double serAdjust = -7;
            float wristIn = (float) ((float) .6 + serAdjust/355);
            wristy.setPosition(wristIn);
            elbow1.setPosition(.91);
            double grabIn = 1;
            double grabOut = .79;
            float servoDegree = 1 / 355;
            telemetry.update();
            if (4 > armAngle + 12 || 4 < armAngle -4) {//  Far and fast arm move into position within an infinite range
                if (4 < armAngle) {
                    Arm1.setPower(.8);
                    Arm2.setPower(.8);
                }
                if (4 > armAngle) {
                    Arm1.setPower(-.7);
                    Arm2.setPower(-.7);
                }
            } else if (4 > armAngle + 5 || 4 < armAngle + 3) { //Close and slow arm move into position if arm is in a 16 degree range
                if (4 < armAngle) {
                    Arm1.setPower(.15);
                    Arm2.setPower(.15);
                }
                if (4 > armAngle) {
                    Arm1.setPower(-.15);
                    Arm2.setPower(-.15);
                }
            } else {
                Arm1.setPower(0);
                Arm2.setPower(0);
                break;
            }
        }


        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == FTCOpenSourceAutonomous.START_POSITION.BLUE_FAR ||
                startPosition == FTCOpenSourceAutonomous.START_POSITION.RED_FAR) {

        }

        switch (startPosition) {
            case BLUE_NEAR:
                //drive = new SampleMecanumDrive(hardwareMap);
                switch(identifiedSpikeMarkLocation){
                    case RIGHT:


                        break;
                    case MIDDLE:

                        break;
                    case LEFT:

                        break;
                }
                /*midwayPose1 = new Pose2d(14, 13, Math.toRadians(45));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(8, 30, Math.toRadians(-90));*/
                break;

            case RED_NEAR:
                drive = new SampleMecanumDrive(hardwareMap);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:

                        break;
                    case MIDDLE:

                        break;
                    case RIGHT:

                        break;
                }
                break;

            case BLUE_FAR:
                //drive = new SampleMecanumDrive(hardwareMap);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:

                        break;
                    case MIDDLE:
                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                .lineTo(new Vector2d(30, -4))
                                .build();
                        drive.followTrajectorySequence(traj1);
                        release();
                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                .back(6)
                                .turn(Math.toRadians(-90))
                                .forward(10)
                                .build();

                        drive.followTrajectorySequence(traj2);

                        traj5 = drive.trajectorySequenceBuilder(traj2.end())
                                .strafeRight(sense.distancechange(33, "right"))
                                .build();
                        drive.followTrajectorySequence(traj5);
                        traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                .forward(sense.distancechange(2, "front"))
                                .build();


                        drive.followTrajectorySequence(traj7);

                        stack1();
                        safeWaitSeconds(.7);
                        stack2();
                        traj3 = drive.trajectorySequenceBuilder(traj7.end())
                                .forward(sense.distancechange(11, "front"))
                                .build();
                        drive.followTrajectorySequence(traj3);

                        in();
                        traj6 = drive.trajectorySequenceBuilder(traj3.end())
                                .strafeLeft(12)
                                .build();
                        drive.followTrajectorySequence(traj6);
                        stopp();
                        pick();

                        traj4 = drive.trajectorySequenceBuilder(traj6.end())
                                .back (80)
                                .turn(Math.toRadians(180))
                                .forward(20)
                                .build();
                        drive.followTrajectorySequence(traj4);
                        traj8 = drive.trajectorySequenceBuilder(traj4.end())
                                .forward(sense.distancechange(28, "front"))
                                .strafeLeft(22)
                                .build();
                        drive.followTrajectorySequence(traj8);
                        armout();
                        traj9 = drive.trajectorySequenceBuilder(traj8.end())
                                .forward(sense.distancechange(8, "front"))
                                .build();
                        drive.followTrajectorySequence(traj9);
                        grabby.setPosition(1);
                        traj10 = drive.trajectorySequenceBuilder(traj9.end())
                                .forward(sense.distancechange(9, "front"))
                                .build();
                        drive.followTrajectorySequence(traj10);
                        grabby.setPosition(.85);
                        traj11 = drive.trajectorySequenceBuilder(traj10.end())
                                .back(1)
                                .strafeRight(4)
                                .build();
                        drive.followTrajectorySequence(traj11);
                        traj12 = drive.trajectorySequenceBuilder(traj11.end())
                                .forward(sense.distancechange(8, "front"))
                                .build();
                        drive.followTrajectorySequence(traj12);
                        wristy.setPosition(0);
                        grabby.setPosition(1);
                        traj13 = drive.trajectorySequenceBuilder(traj12.end())
                                .forward(sense.distancechange(9, "front"))
                                .back(1)
                                .strafeRight(10)
                                .build();
                        drive.followTrajectorySequence(traj13);
                        break;
                    case RIGHT:

                        break;
                }
                /*midwayPose1a = new Pose2d(30, -18, Math.toRadians(90));
                intakeStack = new Pose2d(52, -19,Math.toRadians(90));
                midwayPose2 = new Pose2d(47, -23, Math.toRadians(0));
                midwayPose3 = new Pose2d(48, 0, Math.toRadians(90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliane partner to move from board*/
                break;

            case RED_FAR:
                drive = new SampleMecanumDrive(hardwareMap);
                switch(identifiedSpikeMarkLocation){
                    case RIGHT:

                        break;
                    case MIDDLE:

                        break;
                    case LEFT:

                        break;
                }
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
    private void initTfod() {


        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.8f);

    }    // end method initTfod()

    private void runTfodTensorFlow() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        //Camera placed between Left and Right Spike Mark on RED_FAR and BLUE_NEAR If pixel not visible, assume Right spike Mark
        if (startPosition == FTCOpenSourceAutonomous.START_POSITION.RED_FAR || startPosition == FTCOpenSourceAutonomous.START_POSITION.BLUE_NEAR) {
            identifiedSpikeMarkLocation = FTCOpenSourceAutonomous.IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
        } else { //RED_NEAR or BLUE_FAR
            identifiedSpikeMarkLocation = FTCOpenSourceAutonomous .IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
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
                    identifiedSpikeMarkLocation = FTCOpenSourceAutonomous.IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                } else {
                    identifiedSpikeMarkLocation = FTCOpenSourceAutonomous.IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                }
            } else {
                identifiedSpikeMarkLocation = FTCOpenSourceAutonomous.IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
            }

        }// end for() loop

    }   // end method runTfodTensorFlow()

}   // end class
