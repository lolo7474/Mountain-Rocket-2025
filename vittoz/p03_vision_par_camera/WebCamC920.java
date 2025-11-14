package org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class WebCamC920 {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    public List<AprilTagDetection> detections;
    public List<AprilTag> liste_aprilTag;


    private int nbre_lectures_pour_moyenne = 1;
    public double bearingMax = 22.0;



    /**
     * Constructeur: Classe pour gerer la webcam C920 avec detection AprilTag
     * @param telemetry
     * @param hardwareMap
     */
    public WebCamC920(Telemetry telemetry, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.telemetry = telemetry;

        // 1) Cr�er le processeur AprilTag (famille 36h11 par d�faut)
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        /*

         */

        // 2) Construire le VisionPortal avec la webcam + le processor
        /*
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag);

        visionPortal = portalBuilder.build();
        
         */
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                //.setStreamFormat(VisionPortal.StreamFormat.MJPEG)     // optionnel
                //.setAutoStopLiveView(false)                           // optionnel
                .build();

        liste_aprilTag = new ArrayList<>();


        telemetry.addData("VisionPortal initialisation", "OK");

    }

    /**
     * Démarrer la détection des AprilTag et remplir la liste des AprilTag détectés
     * lit plusieur fois les tags pour faire une moyenne
     */
    public void startDetection() {

        liste_aprilTag = new ArrayList<>();

        // on lance x detection pour faire la moyenne
        for (int i = 0; i < nbre_lectures_pour_moyenne; i++) {

            try {
                //Thread.sleep(100);
                liste_aprilTag.addAll(getDetections());
                //telemetry.addData("Nb tags detecter", liste_aprilTag.size());
            }
            catch (Exception e) {
                telemetry.addData("Erreur ", e.getMessage());
            }
        }

        // on fait la moyenne de chaque valeur des Tag detecter
        //calculerMoyenneDesTagsDetecter();

        // affichage des tags detecter
        for (AprilTag tag : liste_aprilTag) {
            telemetry.addLine(""+tag.toString());
        }
    }


    /**
     * Calculer la moyenne des tags detecter
     * si un tag est detecter plusieurs fois, on fait la moyenne de chaque valeur et on efface les doublons
     */
    public void calculerMoyenneDesTagsDetecter() {


        for (int i = 0; i < liste_aprilTag.size(); i++) {
            AprilTag courant = liste_aprilTag.get(i);
            int nombreDeFoisVue = 1;
            for (int j = i + 1; j < liste_aprilTag.size(); j++) {
                AprilTag autre = liste_aprilTag.get(j);
                if (courant.id == autre.id) {
                    // Modifier le premier élément
                    courant.bearing = courant.bearing +  autre.bearing;
                    courant.range = courant.range + autre.range;
                    courant.elevation += autre.elevation;
                    courant.yaw += autre.yaw;
                    courant.pitch += autre.pitch;
                    courant.roll += autre.roll;

                    nombreDeFoisVue = nombreDeFoisVue + 1;


                    //Supprimer le doublon
                    liste_aprilTag.remove(j);
                    j--; // pour compenser la suppression
                }

                //telemetry.addLine("Tag ID " + courant.id + " vue " + nombreDeFoisVue + " fois");

                courant.bearing = courant.bearing /  nombreDeFoisVue;
                courant.range =   courant.range / nombreDeFoisVue;
                courant.elevation /= nombreDeFoisVue;
                courant.yaw /= nombreDeFoisVue;
                courant.pitch /= nombreDeFoisVue;
                courant.roll /= nombreDeFoisVue;
            }
        }
    }

    /**
     * Récupère les détections actuelles des AprilTag et les convertit en objets AprilTag personnalisés
     * on garde que les tags avec des données métriques
     * @return Liste des AprilTag détectés avec leurs données métriques
     */
    private List<AprilTag> getDetections() {
        detections = aprilTagProcessor.getDetections();

        List<AprilTag> liste_aprilTag_temporaire = new ArrayList<>();

        if (!detections.isEmpty()) {
            //telemetry.addData("Nb tags detecter", detections.size());

            // On affiche les tags detecter
            for (AprilTagDetection tag : detections) {
                try
                {
                    if (tag.ftcPose != null)
                    {
                        AprilTag aprilTag = new AprilTag(tag.id,
                                tag.ftcPose.range ,
                                tag.ftcPose.bearing,
                                tag.ftcPose.elevation,
                                tag.ftcPose.yaw,
                                tag.ftcPose.pitch,
                                tag.ftcPose.roll
                        );
                        //telemetry.addData("range",tag.ftcPose.range);
                        liste_aprilTag_temporaire.add(aprilTag);
                        //telemetry.addLine(aprilTag.toString());

                    }
                    else {

                        AprilTag aprilTag = new AprilTag(tag.id);
                        liste_aprilTag_temporaire.add(aprilTag);
                        //telemetry.addLine(aprilTag.toString());

                    }

                }
                catch (Exception e){
                    telemetry.addData("Erreur lors de la lecture du tag ID " + tag.id, e.getMessage());
                }

            }
        }
        return liste_aprilTag_temporaire;
    }
}
