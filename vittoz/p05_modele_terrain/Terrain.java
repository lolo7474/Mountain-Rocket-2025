package org.firstinspires.ftc.teamcode.vittoz.p05_modele_terrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vittoz.p03_vision_par_camera.AprilTag;

public class Terrain{

    Telemetry telemetry;

    // dimensions du terrain en cm
    double longueur_Terrain = 365.75;  // terrain cecam 356.5
    public double largeur_Terrain = 365.75;  // terrain cecam 356.5
    public double taille_tuille = 60.95;

    // dimensions du but en cm
    double hauteur_But = 98.45;
    double profondeur_But = 46.45;
    double ouverture_But = 67.30;
    double largeur_But = taille_tuille; // TODO ****** * ** * * * * * * * * * * *** * **   ATTENTION ICI, EST CE QUE CEST VRAI � � � � � ! ! ! ! ! ***** * * * * **

    // dimensions de l'AprilTag sur le but en cm
    double hauteur_milieu_AprilTag = hauteur_But - 23.5;
    double position_x_milieu_AprilTag_sur_but = ouverture_But / 2;
    double largeur_AprilTag = 16.50;

    // *****  TODO  il faut verrifier si la largeur du but est la meme qu"une tuille   ***** * * **
	/*

	         BUT bleu                                      BUT rouge                                       Terrain
			                     365.75
			 60.95                                           60.95                              But bleu               But Rouge
			_________________________________________________________							_______________________________
			|	    a /	                                  \a        |							| /     	                \ |
			|	     /                                     \        |							|/	     	                 \|
			|       /                                       \       |							||	                         ||
			|	   /    							         \      |							||	                         ||
			|     /   								          \     |							||	    	                 ||
			|    /     								           \    |							|	     	                  |
			|   /      								            \   |						y	|	     	                  |
			|  |                                                 |  |							|	     	                  |
			|  |                                                 |  |							|	     	                  |
			|  |                                                 |  |							|	     	                  |
			|  |                                                 |  |							|	     	                  |
			|  |                                                 |  |							|	     	                  |
																								|	     	                  |
																								|_____________________________|
																								               x
	*/


    //TODO jai mesuere l'angle sur notre but cecam et ca donne 51 degres
    // a voir si cest pareil sur le terrain officiel
    double angle_a_AprilTag = Math.toRadians(51);// Math.asin(profondeur_But/ largeur_But);  //  =49,64
    double deport_x_aprilTag_dans_terrain = 37;// largeur_But - Math.cos(angle_a_AprilTag) * (ouverture_But /2);
    double y_aprilTag_dans_terrain = longueur_Terrain - 31;// longueur_Terrain - Math.sin(angle_a_AprilTag) * (ouverture_But /2);

    But but_Bleu;
    public But but_Rouge;

    /**
     * Constructeur: Classe pour gerer le terrain
     * creer les 2 but avec les valeurs de leur aprilTag
     * @param telemetry
     */
    public Terrain(Telemetry telemetry) {
        this.telemetry = telemetry;

        but_Bleu = new But (telemetry,"BLEU",20, angle_a_AprilTag, deport_x_aprilTag_dans_terrain, y_aprilTag_dans_terrain);
        but_Rouge = new But (telemetry,"ROUGE",24, angle_a_AprilTag, largeur_Terrain -deport_x_aprilTag_dans_terrain,  y_aprilTag_dans_terrain);


        telemetry.addData("Terrain initialisation", "OK");
    }


    /**
     * calcule la coordonnee du robot dans le terrain en fonction de la detection d'un aprilTag
     * @param detection : detection d'un aprilTag
     * @return coordonnee du robot dans le terrain
     */
    public Coordonnee calculer_Coordonnee_Robot_Dans_Terrain (AprilTag detection)
    {
        Coordonnee coordonneeRobot = new Coordonnee ("",-1, -1);
        try {

            double x_dans_terrain = 0;
            double y_dans_terrain = 0;

            /*
                "Range (m)", d.ftcPose.range
                "Bearing (deg)", d.ftcPose.bearing)
                "Elevation (deg)", d.ftcPose.elevation
                "Yaw (deg)", d.ftcPose.yaw
                "Pitch (deg)", d.ftcPose.pitch
                "Roll (deg)", d.ftcPose.roll
            */

            double angle_yaw_aprilTag = Math.toRadians(detection.yaw);
            double distance_aprilTag = detection.range; // TODO : verifier car je n'ai pas pris en compmte la hauteur de l'apriltag par rapport au sol, sa doit changer la distance

            if (detection.id == but_Bleu.tag_id) {
                // si aprilTag but bleu

                // si angle positif
                if (angle_yaw_aprilTag > 0)
                {
                    double angle = 0;

                    //telemetry.addData("angle_a_AprilTag",Math.toDegrees( angle_a_AprilTag));
                    //telemetry.addData("angle_yaw_aprilTag",Math.toDegrees( angle_yaw_aprilTag));
                    if (angle_yaw_aprilTag < angle_a_AprilTag)
                    {
                        angle = angle_a_AprilTag - angle_yaw_aprilTag;

                        x_dans_terrain = but_Bleu.x_aprilTag_dans_terrain + Math.sin(angle) * distance_aprilTag;
                        y_dans_terrain = but_Bleu.y_aprilTag_dans_terrain - Math.cos(angle) * distance_aprilTag;
                    }
                    else //
                    {
                        angle = angle_yaw_aprilTag - angle_a_AprilTag;
                        x_dans_terrain = but_Bleu.x_aprilTag_dans_terrain - Math.sin(angle) * distance_aprilTag;
                        y_dans_terrain = but_Bleu.y_aprilTag_dans_terrain - Math.cos(angle) * distance_aprilTag;
                    }
                    //telemetry.addData("angle",Math.toDegrees( angle));
                    //telemetry.addData("but_Bleu.x_aprilTag_dans_terrain",but_Bleu.x_aprilTag_dans_terrain);
                    //telemetry.addData("but_Bleu.y_aprilTag_dans_terrain",but_Bleu.y_aprilTag_dans_terrain);
                    //telemetry.addData("x_dans_terrain",x_dans_terrain);
                    //telemetry.addData("y_dans_terrain",y_dans_terrain);


                }
                else // si angle negatif
                {

                    double angle = 0;

                    // si angle plus grand que la vertical
                    if (-angle_yaw_aprilTag < Math.toRadians(90) -angle_a_AprilTag)
                    {
                        angle = Math.toRadians(90) -angle_a_AprilTag - (-angle_yaw_aprilTag) ;
                        x_dans_terrain = but_Bleu.x_aprilTag_dans_terrain + Math.cos(angle) * distance_aprilTag;
                        y_dans_terrain = but_Bleu.y_aprilTag_dans_terrain - Math.sin(angle) * distance_aprilTag;

                    }
                    else
                    {
                        angle = -angle_yaw_aprilTag -(Math.toRadians(90) - angle_a_AprilTag);
                        x_dans_terrain = but_Bleu.x_aprilTag_dans_terrain + Math.cos(angle) * distance_aprilTag;
                        y_dans_terrain = but_Bleu.y_aprilTag_dans_terrain + Math.sin(angle) * distance_aprilTag;
                    }

                }

            }
            else if (detection.id == but_Rouge.tag_id) { // si aprilTag but rouge
                // si angle positif
                if (angle_yaw_aprilTag > 0)
                {
                    double angle = 0;

                    // si angle plus grand que la vertical
                    if (angle_yaw_aprilTag < Math.toRadians(90) - angle_a_AprilTag)
                    {
                        angle = Math.toRadians(90) - angle_a_AprilTag - angle_yaw_aprilTag;
                        x_dans_terrain = but_Rouge.x_aprilTag_dans_terrain - Math.cos(angle) * distance_aprilTag;
                        y_dans_terrain = but_Rouge.y_aprilTag_dans_terrain - Math.sin(angle) * distance_aprilTag;

                    }
                    else
                    {
                        angle = angle_yaw_aprilTag -(Math.toRadians(90)-  angle_a_AprilTag);
                        x_dans_terrain = but_Rouge.x_aprilTag_dans_terrain - Math.cos(angle) * distance_aprilTag;
                        y_dans_terrain = but_Rouge.y_aprilTag_dans_terrain + Math.sin(angle) * distance_aprilTag;

                    }

                }
                else // si angle negatif
                {
                    double angle = 0;

                    // si angle au dessus de l horizontal
                    if (-angle_yaw_aprilTag <   angle_a_AprilTag)
                    {
                        angle = angle_a_AprilTag  - (-angle_yaw_aprilTag);
                        x_dans_terrain = but_Rouge.x_aprilTag_dans_terrain - Math.sin(angle) * distance_aprilTag;
                        y_dans_terrain = but_Rouge.y_aprilTag_dans_terrain - Math.cos(angle) * distance_aprilTag;
                    }
                    else // si angle au dessous de l horizontal
                    {
                        angle = - angle_yaw_aprilTag - angle_a_AprilTag;
                        x_dans_terrain = but_Rouge.x_aprilTag_dans_terrain + Math.sin(angle) * distance_aprilTag;
                        y_dans_terrain = but_Rouge.y_aprilTag_dans_terrain - Math.cos(angle) * distance_aprilTag;
                    }

                }

            }
            else {
                // erreur, AprilTag non reconnu
                return new Coordonnee ("",-1, -1);
            }

            coordonneeRobot = new Coordonnee ("POSITION ROBOT",x_dans_terrain, y_dans_terrain);
            //telemetry.addLine(coordonneeRobot.toString());
            return coordonneeRobot;
        }
        catch (Exception e)
        {
            telemetry.addData("Erreur dans le calcul de la coordonnee du robot dans le terrain", e.getMessage());
            return new Coordonnee ("",-1, -1);
        }

    }

/*

	         BUT bleu                                      BUT rouge
			_________________________________________________________
			|	      /	                                  \         |
			|	     /                                     \        |
			|       /                                       \       |
			|	   /    							         \      |
			|     /   								          \     |
			|    /     								           \    |
			|   /      								            \   |
			|  |                                                 |  |
			|  |                                                 |  |
			|  |                                                 |  |
			|  |                                                 |  |
			|__|                                                 |__|
			|                                                       |
			|                                                       |
			|                                                       |
			|                                                       |
			|                                                       |
			|                                                       |
			|_______________________________________________________|

                               y ↑
                                 |
                          +135°  |  +45°
                                 |
                        ← -180°  +----------→  0° (axe X)
                                 |
                          -135°  |  -45°
                                 |


		*/
    /**
     * calcule l'angle du robot dans le terrain en fonction de la detection d'un aprilTag en radian
     * @param tag : detection d'un aprilTag
     * @param coordonneeDuRobot : coordonnee du robot dans le terrain
     * @return angle du robot dans le terrain
     */
    public double getAngleRobotDansTerrain(AprilTag tag,Coordonnee coordonneeDuRobot) {
        /*
			"Range (m)", d.ftcPose.range
			"Bearing (deg)", d.ftcPose.bearing)
			"Elevation (deg)", d.ftcPose.elevation
			"Yaw (deg)", d.ftcPose.yaw
			"Pitch (deg)", d.ftcPose.pitch
			"Roll (deg)", d.ftcPose.roll
*/


        double angle_horizontal_aprilTag = Math.toRadians(tag.bearing);

        double angleRobotBut = 0;


        double angle_robot = -1;
        if (tag.id == but_Bleu.tag_id) {
            // TODO a verifier si cets bon les atan2
            angleRobotBut = Math.atan2(but_Bleu.y_aprilTag_dans_terrain - coordonneeDuRobot.y_dans_terrain, but_Bleu.x_aprilTag_dans_terrain - coordonneeDuRobot.x_dans_terrain);
        }
        else if (tag.id == but_Rouge.tag_id) {
            // TODO a verifier si cets bon les atan2
            angleRobotBut = Math.atan2(but_Rouge.y_aprilTag_dans_terrain - coordonneeDuRobot.y_dans_terrain, but_Rouge.x_aprilTag_dans_terrain - coordonneeDuRobot.x_dans_terrain);
        }

        angle_robot = angleRobotBut + angle_horizontal_aprilTag;


        return angle_robot;


    }
}
