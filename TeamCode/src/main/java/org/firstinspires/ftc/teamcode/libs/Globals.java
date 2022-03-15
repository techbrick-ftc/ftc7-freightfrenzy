// Holds all of the global variables and setup methods

package org.firstinspires.ftc.teamcode.libs;

import android.content.Context;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

public class Globals {
    private static T265Camera camera;
    private static BNO055IMU imu;
    private static BNO055IMU imu2;
    public static Pose2d endingPose = new Pose2d(0, 0, new Rotation2d(0));

    public static void setupCamera(HardwareMap hardwareMap, Pose2d pose2d) {
        System.out.println("::::setting up t265");
        if (camera == null) {
            camera = new T265Camera(new Transform2d(new Translation2d(-3.5 * 0.0254, 3.75 * 0.0254), new Rotation2d()), 0.1, hardwareMap.appContext);
            System.out.println("::::camera now exists");
            System.out.println(pose2d.relativeTo(endingPose));
            camera.start();
            camera.setPose(pose2d);
            System.out.println("::::finished setting up t265");
        } else {
            System.out.println(pose2d.relativeTo(endingPose));
            startCamera();
            camera.setPose(pose2d);
            System.out.println("::::updated pose to: " + pose2d);
        }
    }

    public static void setupIMU(HardwareMap hardwareMap) {
        if (imu == null) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            imu.initialize(params);
        }

        if (imu2 == null) {
            imu2 = hardwareMap.get(BNO055IMU .class, "imuII");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            imu2.initialize(params);
        }
    }

    public static BNO055IMU getImu() { return imu; }
    public static BNO055IMU getImu2() { return imu2; }
    public static T265Camera getCamera() { return camera; }

    public static void startCamera() {
        System.out.println("::::starting t265");
        if (camera.isStarted()) {
            System.out.println("::::t265 is already started, stopping");
            camera.stop();
        }
        camera.start();
        System.out.println("::::started t256");
    }
    public static void stopCamera() {
        System.out.println("::::stopping t265");
        camera.stop();
        System.out.println("::::stopped t265");
    }
}
