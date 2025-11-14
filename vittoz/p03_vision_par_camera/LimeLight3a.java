package org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Classe pour gerer la Limelight 3a avec detection AprilTag
 */
public class LimeLight3a {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    public List<AprilTag> liste_aprilTag;
    public double bearingMax = 22.0;

    private Limelight3A limelight;

    public double rangeCorrection = ((double) 100); //TODO verifier avec la nouvelle camera
    public double bearingCorrection = (100*(25/11)); //TODO verifier avec la nouvelle camera


    /**
     * Constructeur: Classe pour gerer la Limelight 3a avec detection AprilTag
     * @param telemetry
     * @param hardwareMap
     */
    public LimeLight3a(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        limelight = hardwareMap.get(Limelight3A.class, "limelight3a");

        //telemetry.setMsTransmissionInterval(11);

        liste_aprilTag = new ArrayList<>();
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("LimeLight3a initialisé OK");

    }

    /**
     * Arrêter la détection des AprilTag
     */
    public void stopDetection() {
        limelight.stop();
    }

    /**
     * Démarrer la détection des AprilTag et remplir la liste des AprilTag détectés
     */
    public void startDetection() {

        liste_aprilTag = new ArrayList<>();


        // Récupère la dernière frame traitée par la Limelight
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            // --- Détails par AprilTag détecté
            List<FiducialResult> tags = result.getFiducialResults();


            for (int i = 0; i < tags.size(); i++) {
                FiducialResult t = tags.get(i);
                int id = t.getFiducialId();

                // 1) Robot dans le repère du Tag (souvent le plus utile pour se recaler/driver-to-tag)
                //Pose3D robotInTag = t.getRobotPoseTargetSpace();

                // 2) Caméra dans le repère du Tag
                //Pose3D camInTag = t.getCameraPoseTargetSpace();

                // 3) Tag dans le repère de la caméra
                Pose3D tagInCam = t.getTargetPoseCameraSpace();

                // 4) Tag dans le repère du robot
                //Pose3D tagInRobot = t.getTargetPoseRobotSpace();

                // 5) Robot dans le repère du terrain en se basant sur CE tag uniquement
                Pose3D robotInFieldFromThisTag = t.getRobotPoseFieldSpace();

                // --- Exemples de données télémetries (robot par rapport au tag)
                if (tagInCam != null) {
                    double x = tagInCam.getPosition().x;
                    double y = tagInCam.getPosition().y;
                    double z = tagInCam.getPosition().z;
                    double yaw = tagInCam.getOrientation().getYaw(AngleUnit.DEGREES);
                    double pitch = tagInCam.getOrientation().getPitch(AngleUnit.DEGREES);
                    double roll = tagInCam.getOrientation().getRoll(AngleUnit.DEGREES);

                    telemetry.addLine("x=" + String.format("%.3f", x) + " m , z=" + String.format("%.3f", z) + " m , bearing=" + Math.toDegrees(Math.atan(x/z)) + " ");

                    //telemetry.addLine(String.format("Tag %d -> tagInCam: X=%.3f Y=%.3f Z=%.3f m | Yaw=%.1f Pitch=%.1f Roll=%.1f°",                            id, x, y, z, yaw, pitch, roll));


                    AprilTag aprilTag = new AprilTag(
                            id,         //id
                            z * rangeCorrection,          // range
                            -Math.toDegrees(Math.atan(x/z)),          // bearing
                            y,          // elevation
                            -pitch,     // yaw
                            yaw,        // pitch
                            roll        // roll
                    );
                    liste_aprilTag.add(aprilTag);
                    telemetry.addLine(liste_aprilTag.toString());
                }

            }
        } else {
            telemetry.addLine("Limelight : aucun tag");
        }


    }
}
