    package org.firstinspires.ftc.teamcode.vittoz.p02_deplacement;

    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera.*;
    import org.firstinspires.ftc.teamcode.vittoz.p05_modele_terrain.*;

    import org.firstinspires.ftc.robotcore.external.Telemetry;

    import java.util.ArrayList;
    import java.util.List;

    /**
     * classe pour bouger le robot vers une coordonnee dans le terrain
     * en utilisant les april tag pour se localiser
     * peut se deplacer vers une coordonnee tout en visant un but
     */
    public class Bouger_Robot_Coordonnee_Via_April{
        private final Telemetry telemetry;

        MoteurDeplacement moteur_mouvement;

        List<AprilTag> detections;

        ElapsedTime temps = new ElapsedTime();
        private double tempsDernierTagDetecte;
        private double tempsDernierSetPower;
        private double tempsAcceleration = 1.5; // temps en seconde pour atteindre la vitesse max
        private double tempsMaximumSansTagPourContinuerMouvement = 3.0; // temps en seconde pour continuer le mouvement apres la derniere detection de tag



        Terrain terrain ;

        List<Coordonnee> coordonnees_dans_terrain  = new ArrayList<>();
        private double distanceDeseleration = 40.0; // distance en cm pour commencer a deselerer avant la cible
        double multiplicateurPuissanceMoteurPourAvancer = 1;  // reduit la vitesse max pour atteindre la cible
        double multiplicateurPuissanceMoteurPourTourner = 1;  //reduit la vitesse de rotation pour salligner avec la cible

        private double angleBearingSiAlignerAuBut = 5; // TODO a regler selon les test sur le terrain : est ce que 3° cest suffisant pour considerer que le robot est allignier au but ???
        private double distanceConsidereeAtteinte = 10.0; // distance en cm pour considerer que la coordonnee est atteinte

        double vitesseMini = 0.08;
        private double maxVariationAccelerationParSeconde = 0.5;

        double AV_G = 0;
        double AV_D = 0;
        double AR_G = 0;
        double AR_D = 0;

        double AV_G_prec = 0;
        double  AV_D_prec = 0;
        double  AR_G_prec = 0;
        double  AR_D_prec = 0;


        private boolean allignierAuBut = false;
        private double ancien_angle_deplacementReel;
        private double ancien_distance_vers_coordonnee;

        private double ancien_powerMoteurPourTourner;


        // pour le PID de l'alignement au but
        private double erreurPrecBearing = 0;
        private double sommeErreurBearing = 0;
        private double tempsPrecBearing = 0;


        /**
         * constructeur
         * @param telemetry
         * @param ter // terrain
         * @param moteur_mouv
         */
        public Bouger_Robot_Coordonnee_Via_April(Telemetry telemetry, Terrain ter, MoteurDeplacement moteur_mouv){
            this.telemetry = telemetry;
            terrain = ter;
            moteur_mouvement = moteur_mouv;

            detections = new ArrayList<>();

            //ajout des coordonner dans la liste coordonnees_dans_terrain
            initCoordonneesDansTerrain();
            initBougerToutSeul();

            telemetry.addData("Bouger_Robot_Coordonnee_Via_April initialisation", "OK");

        }

        /**
         * initialiser les coordonnees dans le terrain
         * permet d'ajouter des coordonner dans la liste coordonnees_dans_terrain
         * c'est les destination ou peut aller le robot
         */
        private void initCoordonneesDansTerrain() {

            coordonnees_dans_terrain.add(new Coordonnee("ZONE CHARGEMENT ROUGE", terrain.taille_tuille, terrain.taille_tuille));
            coordonnees_dans_terrain.add(new Coordonnee("ZONE CHARGEMENT BLEU", terrain.largeur_Terrain - terrain.taille_tuille, terrain.taille_tuille));
            coordonnees_dans_terrain.add(new Coordonnee("BASE ZONE BLEU", 240, 120));
            coordonnees_dans_terrain.add(new Coordonnee("BASE ZONE ROUGE", 120, 120));
            coordonnees_dans_terrain.add(new Coordonnee("AU CENTRE DU TERRAIN", 182.85, 182.85));
            coordonnees_dans_terrain.add(new Coordonnee("DEVANT BUT BLEU", 90, 300));
            coordonnees_dans_terrain.add(new Coordonnee("DEVANT BUT ROUGE", 260, 300));
            coordonnees_dans_terrain.add(new Coordonnee("ZONE LANCEMENT COTE BLEU", terrain.taille_tuille * 3 + terrain.taille_tuille/2, terrain.taille_tuille/2));
            coordonnees_dans_terrain.add(new Coordonnee("ZONE LANCEMENT COTE ROUGE", terrain.taille_tuille * 2 + terrain.taille_tuille/2, terrain.taille_tuille/2));

        }


        /**
         * permet de recuper la coordonnee du robot dans le terrain
         * @return coordonnee du robot dans le terrain
         *
         */
        public Coordonnee get_Coordonnee_Robot_Dans_Terrain() {

            Coordonnee coordonneeDuRobot = null;
            if (detections.isEmpty()) {
                telemetry.addLine("Aucun AprilTag détecté.");
            }
            else
            {
                // On affiche les tags detecter
                for (AprilTag tag : detections) {
                    coordonneeDuRobot = terrain.calculer_Coordonnee_Robot_Dans_Terrain(tag);
                    //telemetry.addData("coordonnee", coordonneeDuRobot.toString());
                }

                return coordonneeDuRobot;
            }
            return coordonneeDuRobot;
        }

        /**
         * permet d'ajouter les april tag detecter par la camera
         */
        public void setAprilTagDetections(List<AprilTag> detections) {
            this.detections.addAll(detections);
        }


        /**
         * bouger le robot selon une coordonnee visee
         * Tout en visant un le but de la meme couleur
         * @param but_visee
         * @param nomDestination
         */
        public boolean bougerRobotvers(But but_visee, String nomDestination ) {
            telemetry.addLine("Bouger vers: " + nomDestination);
            boolean positionAtteinte = false;
            int id_coordonnee_destination = -1;

            // on cherche l'id de la coordonnee destination
            id_coordonnee_destination = rechercherIdCoordonneeDestination(nomDestination);


            // si la coordonnee destination n'est pas trouvee, on s'arrete la
            if (id_coordonnee_destination == -1) {
                telemetry.addData("Erreur", "Coordonnee destination non trouvee: " + nomDestination);
                return false;
            }

            // si pas de tag visible
            if (detections == null || detections.size() ==0) {
                telemetry.addLine("Aucun AprilTag détecté. On bouge à l'aveugle.");
                rotationEtBougerAlaveugle();


            } else {
                //telemetry.addData("Nb tags detecter", detections.size());
                tempsDernierTagDetecte = temps.seconds();

                // On affiche les tags detecter
                for (AprilTag tag : detections) {
                    double powerRotation = 0;
                    // si le tag detecter est celui du but vise
                    if (tag.id == but_visee.tag_id) {
                        // TODO a modifier ici si la camera a bouger avec le servo moteur
                        // on vise lebut
                        powerRotation = getPuissanceMoteurPourViserLeBut(tag);
                        // ICI nouveau calcul de la puiisance de rotation avec PID
                        //powerRotation = getPuissanceMoteurPourViserLeButAvecPID(tag);

                        // on ne bouge pas le robot tant qu'il n'est pas allignier au but
                        //if (!allignierAuBut) return false;
                        //telemetry.addLine("allignierAuBut : " + allignierAuBut  );
                    }



                    // on calcule la position du robot dans le terrain
                    Coordonnee coordonneeDuRobot = terrain.calculer_Coordonnee_Robot_Dans_Terrain(tag);
                    //telemetry.addData("id_coordonnee_destination", coordonnees_dans_terrain.get(id_coordonnee_destination));
                    //telemetry.addData("coordonneeDuRobot", coordonneeDuRobot.toString());

                    // on deplace le robot vers la coordonnee destination si la coordonnee du robot est valide
                    if (coordonneeDuRobot != null && coordonneeDuRobot.x_dans_terrain != -1 && coordonneeDuRobot.y_dans_terrain != -1) {
                        positionAtteinte = deplacerRobotVersCoordonnee(coordonneeDuRobot, coordonnees_dans_terrain.get(id_coordonnee_destination), tag, powerRotation);
                    }

                }
            }

            return positionAtteinte;

        }

        /**
         * rechercher l'id de la coordonnee destination dans la liste coordonnees_dans_terrain
         * @param nomDestination
         * @return
         */
        private int rechercherIdCoordonneeDestination(String nomDestination) {
            // on cherche l'id de la coordonnee destination
            for (int i = 0; i < coordonnees_dans_terrain.size(); i++) {
                if (coordonnees_dans_terrain.get(i).nom.equals(nomDestination)) {
                    return i;
                }
            }
            return -1;
        }

        /**
         * continuer le mouvement precedent si on a perdu le tag
         * continue le mouvement precedent avec une attenuation progressive
         * si on a pas detecter de tag depuis plus de tempsMaximumSansTagPourContinuerMouvement
         * on tourne sur place pour retrouver un tag apres
         */
        private void rotationEtBougerAlaveugle() {
            //telemetry.addLine("temps.seconds()"+temps.seconds());
            //telemetry.addLine("tempsDernierTagDetecte"+tempsDernierTagDetecte);
            //telemetry.addLine("tempsMaximumSansTagPourContinuerMouvement"+tempsMaximumSansTagPourContinuerMouvement);

            if (tempsDernierTagDetecte != -1)
            {
                // si on a detecter un tag il y a moins de 2 seconde on continue le mouvement precedent
                if (temps.seconds() - tempsDernierTagDetecte < tempsMaximumSansTagPourContinuerMouvement)
                {

                        // Décroissance linéaire du mouvement
                        double attenuation = 1.0 - tempsMaximumSansTagPourContinuerMouvement /tempsDernierTagDetecte ;
                        AV_G *= attenuation;
                        AV_D *= attenuation;
                        AR_G *= attenuation;
                        AR_D *= attenuation;

                    // appliquer la puissance aux moteurs
                    moteur_mouvement.setPower(AV_G,AR_G,AV_D,AR_D);
                    return;


                }
                else
                {
                    //on tourne jusqua l'infini pour retrouver un tag
                    // TODO a voir si on met un servo pour la hauteur
                    moteur_mouvement.setPower(multiplicateurPuissanceMoteurPourTourner/2,multiplicateurPuissanceMoteurPourTourner/2,-multiplicateurPuissanceMoteurPourTourner/2,-multiplicateurPuissanceMoteurPourTourner/2);
                    initBougerToutSeul();
                }

            }
        }

        /**
         * deplacer le robot vers une coordonnee dans le terrain :
         * calcule angle_direction_a_prendre
         * calcule de l'angle du robot
         * calcule de l'angle de deplacement
         * evite les changement de puissance brusque dans les moteurs
         * @param coordonneeDuRobot
         * @param coordonnee_destination
         * @param tag
         */
        private boolean deplacerRobotVersCoordonnee(Coordonnee coordonneeDuRobot, Coordonnee coordonnee_destination,AprilTag tag,double rotation) {

            boolean positionAtteinte = false;

            // ******** on calcule langle de deplacement a prendre ********
            // on calcule l'angle a prendre pour aller vers la coordonnee destination
            double angle_direction_a_prendre = get_angle_direction_a_prendre(coordonneeDuRobot,coordonnee_destination);
            //telemetry.addLine("angle_direction_a_prendre: " + Math.toDegrees(angle_direction_a_prendre));

            // on calcule l'angle du robot dans le terrain
            double angle_robot = terrain.getAngleRobotDansTerrain(tag,coordonneeDuRobot);
            //telemetry.addLine("angle_robot: " + Math.toDegrees(angle_robot));

            // on calcule l'angle de deplacement reel a prendre
            double angle_deplacementReel = angle_direction_a_prendre - angle_robot;
            //telemetry.addLine("angle_deplacementReel avant normalisation: " + Math.toDegrees(angle_deplacementReel));

            // lissage de l'angle de deplacement reel
            if (ancien_angle_deplacementReel != -1)
            {
                // on lisse l'angle de deplacement reel pour eviter les changement brusque
                // TODO a voir si je laisse ce lissage ou pas
                //angle_deplacementReel = (angle_deplacementReel + ancien_angle_deplacementReel) / 2;
                //telemetry.addLine("angle_deplacementReel apres lissage: " + Math.toDegrees(angle_deplacementReel));
            }
            ancien_angle_deplacementReel = angle_deplacementReel;

            //telemetry.addData("angle_robot", Math.toDegrees(angle_robot));
            //telemetry.addData("angle_deplacementReel", Math.toDegrees(angle_deplacementReel));


            // ******** on calcule la vitesse a donner aux moteurs pour aller vers la coordonnee destination ********
            double vitessemax;

            // calcule de la distance en trigo
            double distance_vers_coordonnee = Math.abs(Math.abs(coordonnee_destination.y_dans_terrain - coordonneeDuRobot.y_dans_terrain)/Math.sin(angle_direction_a_prendre));
            // calcule de la distance avec pythagore
            distance_vers_coordonnee = Math.hypot(Math.abs(coordonnee_destination.x_dans_terrain - coordonneeDuRobot.x_dans_terrain), Math.abs(coordonnee_destination.y_dans_terrain - coordonneeDuRobot.y_dans_terrain));
            telemetry.addData("distance_vers_coordonnee", distance_vers_coordonnee);

            // on lisse la distance vers la coordonnee pour eviter les changement brusque
            if (ancien_distance_vers_coordonnee != -1)
            {
                // TODO a voir si je laisse ce lissage ou pas
                //distance_vers_coordonnee = (distance_vers_coordonnee + ancien_distance_vers_coordonnee) / 2;
            }
            ancien_distance_vers_coordonnee = distance_vers_coordonnee;



            // on ajuste la vitesse max en fonction de la distance vers la coordonnee destination
            if (distance_vers_coordonnee < distanceConsidereeAtteinte)  // si on est a moins de X cm de la cible
            {
                vitessemax = 0;
                telemetry.addLine("*************** Position atteinte ***************");
                positionAtteinte =  true;

            }
            else if (distance_vers_coordonnee>distanceDeseleration) // si on est loin de la cible
            {
                vitessemax = 2;  // la vitesse max est reguler plus loin par le multiplicateurPuissanceMoteurPourAvancer
            }
            else {
                // on deselere en approchant de la cible en ayant au moin la vitesse mini
                vitessemax = vitesseMini + (1-vitesseMini) * (distance_vers_coordonnee / distanceDeseleration);
                //   1 =           0.02    +     0.98 *     1
                // 0.02 =         0.02    +     0.98 *    0
            }


            // on ajuste la vitesse max en fonction du temps d'acceleration
            if (temps.seconds() < tempsAcceleration)
            {
                vitessemax = vitessemax * (temps.seconds() / tempsAcceleration);
            }

            // on calcule la puissance des moteurs en limitant la variation de puissance
            calculPuissanceMoteurEnLimitantVariation(angle_deplacementReel,vitessemax,rotation);



            // appliquer la puissance aux moteurs si la position n'est pas atteinte
            if (!positionAtteinte)
                moteur_mouvement.setPower(AV_G,AR_G,AV_D,AR_D);
            else
                moteur_mouvement.setPower(0,0,0,0);

            tempsDernierSetPower = temps.seconds();

            return positionAtteinte;


        }

        /**
         * calcul de la puissance des moteurs en limitant la variation de puissance
         * @param angle_deplacementReel
         * @param vitessemax
         * @param rotation
         */
        private void calculPuissanceMoteurEnLimitantVariation(double angle_deplacementReel, double vitessemax, double rotation)
        {

            boolean puissanceOK = false;
            double avG = 0, avD = 0, arG = 0, arD = 0;

            double x = 0;
            double y  = 0;

            double dt = temps.seconds() - tempsDernierSetPower;
            double deltaMax = maxVariationAccelerationParSeconde * dt;

            telemetry.addLine("vitessemax avant: " + vitessemax);
            for (int i = 0; i < 1000; i++)
            {
                x = vitessemax * Math.sin(angle_deplacementReel);
                y = vitessemax * Math.cos(angle_deplacementReel);
                avG = (y - x ) * multiplicateurPuissanceMoteurPourAvancer + rotation;
                avD = (y + x ) * multiplicateurPuissanceMoteurPourAvancer - rotation;
                arG = (y + x ) * multiplicateurPuissanceMoteurPourAvancer + rotation;
                arD = (y - x ) * multiplicateurPuissanceMoteurPourAvancer - rotation;

                //on verifie si la variation de puissance depasse le deltaMax
                if (avG > AV_G_prec + deltaMax  ||
                        avD > AV_D_prec + deltaMax ||
                        arG > AR_G_prec + deltaMax ||
                        arD > AR_D_prec + deltaMax)
                {
                    // reduire la vitessemax et rotation
                    vitessemax = vitessemax -  vitessemax / 1000;
                    rotation = rotation - rotation / 1000;
                }



                else
                {
                    puissanceOK = true;
                    break;
                }

            }

            // on enregistre les puissances finales
            AV_G = avG;
            AV_D = avD;
            AR_G = arG;
            AR_D = arD;

            // on sauvegarde les puissances precedentes
            AV_G_prec = AV_G;
            AV_D_prec = AV_D;
            AR_G_prec = AR_G;
            AR_D_prec = AR_D;


        }

        /**
         * calcul de l'angle a prendre pour aller vers la coordonnee destination en radian
         *  ! ! ! ne prend pas en compte l'orientation du robot ! ! !
         *  seulement la position actuelle de la position de destination
         * @param coordonnee_robot
         * @param coordonnee_destination
         * @return
         */
        private double get_angle_direction_a_prendre(Coordonnee coordonnee_robot, Coordonnee coordonnee_destination) {

            // calcul des écarts entre la position actuelle du robot et la destination
            double difference_x = coordonnee_destination.x_dans_terrain - coordonnee_robot.x_dans_terrain;
            double difference_y = coordonnee_destination.y_dans_terrain - coordonnee_robot.y_dans_terrain;


            // calcul de l’angle absolu entre la position du robot et la destination
            // 0 rad = axe X positif (droite), rotation dans le sens antihoraire
            double angle_direction_a_prendre = Math.atan2(difference_y, difference_x);
            /**
             * ┌──────────────────────────────────────────────────────────┐
             * │                    REPÈRE DES ANGLES                     │
             * │                                                          │
             * │             (π/2)  ↑ 90° (vers le haut)                  │
             * │                    │                                     │
             * │                    │                                     │
             * │                    │                                     │
             * │ (π) 180° ←─────────●─────────→ 0 rad (droite, +X)        │
             * │                    │                                     │
             * │                    │                                     │
             * │                    ↓                                     │
             * │           (3π/2)  270° (vers le bas)                     │
             * │                                                          │
             * │ ➤ Les angles augmentent dans le sens anti-horaire        │
             * │ ➤ 0 rad sur l’axe X positif                              │
             * │ ➤ Résultat renvoyé entre [0, 2π[                         │
             * └──────────────────────────────────────────────────────────┘
             */

            // normalisation pour obtenir un angle entre [0, 2π[
            if (angle_direction_a_prendre < 0)
                // on ajoute un tour
                angle_direction_a_prendre += 2 * Math.PI;

            return angle_direction_a_prendre;
        }


        /**
         * get la puissance moteur pour faire tourner le robot sur lui meme pour viser le but
         * @param tag
         */
        private double getPuissanceMoteurPourViserLeBut(AprilTag tag) {
            // on verifi si le robot est allignier au but
            if (Math.abs(tag.bearing) < angleBearingSiAlignerAuBut)
            {
                allignierAuBut = true;
                telemetry.addLine("Robot allignier au but");
                return 0;
            }

            allignierAuBut = false;

            // on calcule la puissance a donner aux moteurs en fonction du bearing pour tourner vers le but
            double powerMoteurPourTourner = tag.bearing / 22 * multiplicateurPuissanceMoteurPourTourner;
            telemetry.addData("powerMoteurPourTourner",powerMoteurPourTourner);

            ancien_powerMoteurPourTourner = powerMoteurPourTourner;

            return powerMoteurPourTourner;

        }

        /**
         * faire tourner le robot sur lui meme pour viser le but avec un PID
         * jamais essayer vraiment, ca beuguait, mais cest surement lavenir
         * @param tag
         */
        private double getPuissanceMoteurPourViserLeButAvecPID(AprilTag tag) {
            double bearing = tag.bearing; // en degrés
            double maintenant = temps.seconds();
            double dt = maintenant - tempsPrecBearing;
            if (dt <= 0) dt = 0.02;

            if (Math.abs(bearing) < angleBearingSiAlignerAuBut) {
                allignierAuBut = true;
                bearing = 0;
            } else {
                allignierAuBut = false;
            }

            // PID simple


            //sensibilité de visée
            //Réagit instantanément à l’erreur actuelle
            double Kp = 0.02;               //proportionnel

            //Corrige les erreurs résiduelles lentes
            double Ki = 0.0001;              //intégral          // a ajuster de tres peu, genre +- 0.0005

            //Freine / amortit les oscillations
            // amorti des oscillations
            double Kd = 0.002;              //dérivé


            sommeErreurBearing += bearing * dt;
            double derivee = (bearing - erreurPrecBearing) / dt;

            double puissance = Kp * bearing + Ki * sommeErreurBearing + Kd * derivee;

            // Clamp de sécurité
            double puissanceMax = 0.5;
            puissance = Math.max(-puissanceMax, Math.min(puissance, puissanceMax));

            erreurPrecBearing = bearing;
            tempsPrecBearing = maintenant;

            ancien_powerMoteurPourTourner = puissance;

            return puissance;
        }

        public void viderListDetection() {
            detections = new ArrayList<>();
        }

        /**
         * Mise a zero des variables pour un nouveau deplacement
         */
        public void initBougerToutSeul() {

            temps.reset();
            allignierAuBut = false;
            ancien_angle_deplacementReel = -1;
            ancien_distance_vers_coordonnee = -1;
            tempsDernierTagDetecte = -1;
            ancien_powerMoteurPourTourner = 0;
            AR_D = 0;
            AR_G = 0;
            AV_D = 0;
            AV_G = 0;

        }


    }
