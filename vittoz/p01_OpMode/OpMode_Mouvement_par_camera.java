package org.firstinspires.ftc.teamcode.vittoz.p01_OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.vittoz.p02_deplacement.*;
import org.firstinspires.ftc.teamcode.vittoz.p05_modele_terrain.*;


import org.firstinspires.ftc.teamcode.vittoz.p04_lanceur.Lanceur;
import org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera.LimeLight3a;
import org.firstinspires.ftc.teamcode.vittoz.p02_deplacement.MoteurDeplacement;
import org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera.ServoRotatLimeLight;
import org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera.WebCamC920;

@TeleOp(name = "VITTOZ Mouvement_par_camera")
/**
 * OpMode teleop pour faire bouger le robot via la camera ou avec les joysticks
 */
public class OpMode_Mouvement_par_camera extends LinearOpMode {

    // pour selectionner les accessoires connectes au robot
    private boolean moteurConnecter = true;
    private boolean lanceurConnecter = false;
    private boolean camera1Connecter = true;
    private boolean limeLight3aConnecter = false;
    private boolean servoRotatLimeLightConnecter = false;


    // variables des modules connectés
    public Terrain terrain;

    MoteurDeplacement moteurDeplacement = null;
    WebCamC920 detection_AprilTag = null;
    LimeLight3a limeLight3a = null;
    Lanceur lanceur = null;
    ServoRotatLimeLight servoRotatLimeLight = null;
    Bouger_Robot_Coordonnee_Via_April bouger_Robot_Coordonnee_Via_April = null;
    boolean bougerToutSeul;





    @Override
    public void runOpMode() {

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

        // variables pour les joysticks
        float joystick_G_y;
        float joystick_G_x;
        float joystick_D_x;

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

            bougerToutSeul = false;

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





            // si on appuie sur le triggers et une touche, on fait bouger le robot vers la coordonnee qu'on a choisi

            if (gamepad1.left_bumper && gamepad1.a) {
                // on fait bouger le robot vers la coordonnee BASE ZONE ROUGE
                bougerToutSeul = true;
                bouger_Robot_Coordonnee_Via_April.bougerRobotvers(terrain.but_Rouge, "BASE ZONE ROUGE");
            }
            else if (gamepad1.left_bumper && gamepad1.y) {
                // on fait bouger le robot vers la coordonnee AU CENTRE DU TERRAIN
                bougerToutSeul = true;
                bouger_Robot_Coordonnee_Via_April.bougerRobotvers(terrain.but_Rouge, "AU CENTRE DU TERRAIN");
            }
            else if (gamepad1.left_bumper && gamepad1.b) {
                // on fait bouger le robot vers la coordonnee AU CENTRE DU TERRAIN
                bougerToutSeul = true;
                bouger_Robot_Coordonnee_Via_April.bougerRobotvers(terrain.but_Rouge, "BASE ZONE BLEU");
            }
            else if (gamepad1.left_bumper && gamepad1.x) {
                // on fait bouger le robot vers la coordonnee AU CENTRE DU TERRAIN
                bougerToutSeul = true;
                bouger_Robot_Coordonnee_Via_April.bougerRobotvers(terrain.but_Rouge, "DEVANT BUT BLEU");
            }
            else
            {
                bouger_Robot_Coordonnee_Via_April.initBougerToutSeul();
            }


            if (servoRotatLimeLightConnecter)
            {
                // suis le premier tag detecter
                    servoRotatLimeLight.folow(limeLight3a.liste_aprilTag.get(0).bearing,limeLight3a.bearingMax);
            }




            // afficher de la position du robot et de l'angle du robot
            /*
            if (limeLight3aConnecter) {

                for (AprilTag tag : limeLight3a.liste_aprilTag) {
                    Coordonnee coordonneeDuRobot = terrain.calculer_Coordonnee_Robot_Dans_Terrain(tag);
                    telemetry.addLine(coordonneeDuRobot.toString());

                    double angleRobot = terrain.getAngleRobotDansTerrain(tag,coordonneeDuRobot);
                    telemetry.addData("Angle robot: ",Math.toDegrees(angleRobot));
                }
            }
            // afficher de la position du robot et de l'angle du robot
            if (camera1Connecter) {
                for (AprilTag tag : detection_AprilTag.liste_aprilTag) {
                    Coordonnee coordonneeDuRobot = terrain.calculer_Coordonnee_Robot_Dans_Terrain(tag);
                    telemetry.addLine(coordonneeDuRobot.toString());

                    double angleRobot = terrain.getAngleRobotDansTerrain(tag,coordonneeDuRobot);
                    telemetry.addData("Angle robot: ",Math.toDegrees(angleRobot));
                }
            }

             */


            // on fait fonctionner le lanceur via les triggers droit
            if (lanceurConnecter) {
                lanceur.lancer(gamepad1.right_trigger);
            }

            // si on ne fait pas bouger le robot tout seul, on utilise les joysticks
            if (!bougerToutSeul) {

                // Lecture des joysticks
                joystick_G_y = -gamepad1.left_stick_y;
                joystick_G_x = gamepad1.left_stick_x;
                joystick_D_x = -gamepad1.right_stick_x;

                // Calcul de la puissance des moteurs avec les joysticks
                powerMoteur_AV_G = joystick_G_y + joystick_G_x + joystick_D_x;
                powerMoteur_AV_D = (joystick_G_y - joystick_G_x) - joystick_D_x;
                powerMoteur_AR_G = (joystick_G_y - joystick_G_x) + joystick_D_x;
                powerMoteur_AR_D = (joystick_G_y + joystick_G_x) - joystick_D_x;


                // Diviser la puissance des moteurs en appuyant
                if (gamepad1.right_bumper) {
                    powerMoteur_AV_G /= 3;
                    powerMoteur_AV_D /= 3;
                    powerMoteur_AR_G /= 3;
                    powerMoteur_AR_D /= 3;
                }

                // Appliquer la puissance aux moteurs
                if (moteurConnecter) {
                    moteurDeplacement.setPower(powerMoteur_AV_G, powerMoteur_AR_G, powerMoteur_AV_D, powerMoteur_AR_D);
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

        // fin du mode operationnel, on arrete tout ce qui doit l'etre
        if (limeLight3aConnecter) {
            limeLight3a.stopDetection();
        }
        if (servoRotatLimeLightConnecter)
        {
            servoRotatLimeLight.initServo();
        }
    }
}

