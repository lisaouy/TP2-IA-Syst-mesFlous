
#include <stdio.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64
#define NB_CAPTEURS 8

#define VMAX 6.28
#define VBASE 1.8

/* Seuils initiaux a calibrer */
#define MUR_LOIN 50.0
#define MUR_CIBLE 150.0
#define MUR_PROCHE 350.0

#define OBSTACLE_AVANT 120.0
#define OBSTACLE_URGENT 350.0

double cap[NB_CAPTEURS];

double min2(double a, double b) {
  return a < b ? a : b;
}

double max2(double a, double b) {
  return a > b ? a : b;
}

double limiter(double x, double a, double b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

/* Fonctions d'appartenance */

double loin(double x) {
  return limiter((MUR_CIBLE - x) /
                 (MUR_CIBLE - MUR_LOIN), 0, 1);
}

double correct(double x) {
  if (x <= MUR_LOIN || x >= MUR_PROCHE)
    return 0;

  if (x <= MUR_CIBLE)
    return (x - MUR_LOIN) /
           (MUR_CIBLE - MUR_LOIN);

  return (MUR_PROCHE - x) /
         (MUR_PROCHE - MUR_CIBLE);
}

double proche(double x) {
  return limiter((x - MUR_CIBLE) /
                 (MUR_PROCHE - MUR_CIBLE), 0, 1);
}

/*
 * Controleur flou de type Sugeno.
 * Sortie : correction de vitesse.
 *
 * Correction positive : tourner a gauche.
 * Correction negative : tourner a droite.
 */

double controleur_flou(double distance,
                       double avant_droit,
                       double arriere_droit) {

  double mu_loin = loin(distance);
  double mu_correct = correct(distance);
  double mu_proche = proche(distance);

  /* Orientation relative au mur */
  double difference = avant_droit - arriere_droit;

  double vers_mur =
    limiter(difference / 250.0, 0, 1);

  double hors_mur =
    limiter(-difference / 250.0, 0, 1);

  /*
   * Regles :
   *
   * R1 : SI mur loin
   *      ALORS tourner a droite.
   *
   * R2 : SI distance correcte
   *      ALORS aller tout droit.
   *
   * R3 : SI mur proche
   *      ALORS tourner a gauche.
   *
   * R4 : SI orientation vers mur
   *      ALORS corriger a gauche.
   *
   * R5 : SI orientation hors mur
   *      ALORS corriger a droite.
   */

  double poids =
      mu_loin + mu_correct + mu_proche
      + vers_mur + hors_mur;

  if (poids < 0.0001)
    return 0;

  double sortie =
      mu_loin * (-1.0)
      + mu_correct * 0.0
      + mu_proche * 1.0
      + vers_mur * 0.7
      + hors_mur * (-0.7);

  return limiter(sortie / poids, -1, 1);
}

int main(int argc, char **argv) {

  wb_robot_init();

  WbDeviceTag gauche =
    wb_robot_get_device("left wheel motor");

  WbDeviceTag droite =
    wb_robot_get_device("right wheel motor");

  wb_motor_set_position(gauche, INFINITY);
  wb_motor_set_position(droite, INFINITY);

  wb_motor_set_velocity(gauche, 0);
  wb_motor_set_velocity(droite, 0);

  WbDeviceTag ps[NB_CAPTEURS];

  for (int i = 0; i < NB_CAPTEURS; i++) {
    char nom[8];
    sprintf(nom, "ps%d", i);

    ps[i] = wb_robot_get_device(nom);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  int compteur = 0;

  while (wb_robot_step(TIME_STEP) != -1) {

    for (int i = 0; i < NB_CAPTEURS; i++) {
      cap[i] = wb_distance_sensor_get_value(ps[i]);
    }

    /* Detection devant */
    double frontal = max2(cap[0], cap[7]);

    double diagonales = max2(cap[1], cap[6]);

    double obstacle =
      max2(frontal, diagonales);

    /* Distance et orientation du mur droit */
    double mur_droit = cap[2];

    double correction = controleur_flou(
      mur_droit, cap[1], cap[3]
    );

    double vg;
    double vd;

    /* PRIORITE 1 : obstacle imminent */
    if (obstacle > OBSTACLE_URGENT) {

      /* Rotation sur place vers la gauche */
      vg = -1.5;
      vd = 1.5;

    } else if (obstacle > OBSTACLE_AVANT) {

      /* PRIORITE 2 : ralentir et tourner */
      double intensite = limiter(
        (obstacle - OBSTACLE_AVANT) /
        (OBSTACLE_URGENT - OBSTACLE_AVANT),
        0, 1
      );

      vg = 0.8 - 1.2 * intensite;
      vd = 2.0 - 0.3 * intensite;

    } else {

      /* PRIORITE 3 : suivre le mur */
      vg = VBASE - 1.2 * correction;
      vd = VBASE + 1.2 * correction;
    }

    vg = limiter(vg, -VMAX, VMAX);
    vd = limiter(vd, -VMAX, VMAX);

    wb_motor_set_velocity(gauche, vg);
    wb_motor_set_velocity(droite, vd);

    if (compteur % 10 == 0) {
      printf(
        "ps0=%.0f ps1=%.0f ps2=%.0f "
        "ps3=%.0f ps6=%.0f ps7=%.0f "
        "| VG=%.2f VD=%.2f\n",
        cap[0], cap[1], cap[2],
        cap[3], cap[6], cap[7],
        vg, vd
      );
    }

    compteur++;
  }

  wb_robot_cleanup();
  return 0;
}
