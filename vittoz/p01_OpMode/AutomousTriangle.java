package org.firstinspires.ftc.teamcode.vittoz.p01_OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vittoz.p02_deplacement.*;
import org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera.*;
import org.firstinspires.ftc.teamcode.vittoz.p04_lanceur.*;
import org.firstinspires.ftc.teamcode.vittoz.p05_modele_terrain.*;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "VITTOZ fait un triangle", group = "Vittoz")
/**
 * OpMode autonome pour faire un triangle dans le terrain
 */
public class AutomousTriangle extends LinearOpMode {


    // pour selectionner les accessoires connectes au robot
    private boolean moteurConnecter = true;
    private boolean lanceurConnecter = false;
    private boolean camera1Connecter = false;
    private boolean limeLight3aConnecter = true;
    private boolean servoRotatLimeLightConnecter = false;



    // variables des modules
    Terrain terrain;

    MoteurDeplacement moteurDeplacement = null;
    WebCamC920 detection_AprilTag = null;
    LimeLight3a limeLight3a = null;
    Lanceur lanceur = null;
    ServoRotatLimeLight servoRotatLimeLight = null;
    Bouger_Robot_Coordonnee_Via_April bouger_Robot_Coordonnee_Via_April = null;


    List<String> listeDestinations = new ArrayList<>();


    int iDestinationEnCours = 0;



    private boolean destinationAttein = false;


    @Override
    public void runOpMode() throws InterruptedException {

        listeDestinations.add("BASE ZONE ROUGE");
        listeDestinations.add("BASE ZONE BLEU");
        listeDestinations.add("DEVANT BUT BLEU");



        //initialisation des modules

        terrain = new Terrain(telemetry);

        if (moteurConnecter) {
            moteurDeplacement = new MoteurDeplacement(telemetry, hardwareMap);
        }
        else
        {
            moteurDeplacement = null;
        }

        if (lanceurConnecter)
        {
            lanceur = new Lanceur(telemetry, hardwareMap);
        }
        else
        {
            lanceur = null;
        }

        if (limeLight3aConnecter)
        {
            limeLight3a = new LimeLight3a(telemetry, hardwareMap);
        }

        if (camera1Connecter)
        {
            detection_AprilTag = new WebCamC920(telemetry, hardwareMap);
        }
        if (servoRotatLimeLightConnecter)
        {
            servoRotatLimeLight = new ServoRotatLimeLight(telemetry, hardwareMap);
        }


        bouger_Robot_Coordonnee_Via_April = new Bouger_Robot_Coordonnee_Via_April(telemetry, terrain, moteurDeplacement);



        //pour l'affichage du temps d'execution
        ElapsedTime runtime;
        runtime = new ElapsedTime();


        // variables pour la puissance des moteurs
        float powerMoteur_AV_G;
        float powerMoteur_AV_D;
        float powerMoteur_AR_G;
        float powerMoteur_AR_D;

        // Affichage de la tension de la batterie
        double voltage = 0.0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > voltage) { // on garde la plus haute mesure
                voltage = v;
            }
        }
        telemetry.addData("Batterie (V)", String.format("%.2f", voltage));


        telemetry.addData("Robot", "initialisé");
        telemetry.addData("waitForStart...", "");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            telemetry.addData("Status", "Run Time: " + runtime);





            // on lance la detection des aprilTag et on fourni les resultats au module de mouvement par coordonnee
            if (camera1Connecter) {
                detection_AprilTag.startDetection();
                bouger_Robot_Coordonnee_Via_April.setAprilTagDetections(detection_AprilTag.liste_aprilTag);
            }
            if (limeLight3aConnecter) {
                limeLight3a.startDetection();
                bouger_Robot_Coordonnee_Via_April.setAprilTagDetections(limeLight3a.liste_aprilTag);
            }





            String destination = listeDestinations.get(iDestinationEnCours);

            destinationAttein = bouger_Robot_Coordonnee_Via_April.bougerRobotvers(terrain.but_Rouge, destination);

            if (destinationAttein) {
                iDestinationEnCours = (iDestinationEnCours + 1) ;

                if (iDestinationEnCours >= listeDestinations.size()) {
                    iDestinationEnCours = 0;
                }
            }









            // on fait bouger le robot
            if (moteurConnecter) {
                moteurDeplacement.avancerRobot();
            }



            telemetry.update();

            // on vide la liste des detection pour la prochaine boucle
            bouger_Robot_Coordonnee_Via_April.viderListDetection();

            // TODO a voir si il faut laisser ca ou pas
            sleep(50);

        }

        if (limeLight3aConnecter) {
            limeLight3a.stopDetection();
        }
    }
}

