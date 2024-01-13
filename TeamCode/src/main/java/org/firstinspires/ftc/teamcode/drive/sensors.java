package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class sensors {
    private DistanceSensor sensorDistance;
    private DistanceSensor sensorDistance2;
    private DistanceSensor sensorDistancer;
    private DistanceSensor sensorDistancel;
    private DistanceSensor sensorDistanceb;

    public sensors(HardwareMap hardwareMap){

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensorDistance2");
        sensorDistancel = hardwareMap.get(DistanceSensor.class, "dis_left");
        sensorDistancer = hardwareMap.get(DistanceSensor.class, "dis_right");
        sensorDistanceb = hardwareMap.get(DistanceSensor.class, "dis_rear");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) sensorDistance2;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d poseEstimate = drive.getPoseEstimate();
    }

    public double dis(String side) {
        double dis = 1;
        if (side == "front") {
            dis = (sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH)) / 2;
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
}
