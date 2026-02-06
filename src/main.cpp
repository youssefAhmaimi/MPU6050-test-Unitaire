#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <String.h>

Adafruit_MPU6050 mpu;

float angle,TetaG,TetaW;
float gyro;
char FlagCalcul = 0;
float TetaWF,TetaGF,Ve, Vs = 0;
float Te = 10;    // période d'échantillonage en ms
float Tau = 1000; // constante de temps du filtre en ms
float A, B;// coefficient du filtre

void controle(void *parameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
   
    TetaG=-atan2(a.acceleration.y, a.acceleration.x); // Angle du gyroscope en radian
    TetaGF = A * TetaG + B * TetaGF;                  // Angle du gyroscope filtré en radian
   
    TetaW  =  g.gyro.z * Tau/1000;                    // angle de l'accélération en radian  
    TetaWF = A* TetaW + B*TetaWF;                     // angle de l'accélération filtré en radian
 
    angle = TetaWF + TetaGF;                          // angle de l'inclinaison du gyropode en radian

    FlagCalcul = 1;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

void Vin(void *parameters)
{
  Ve = 1;
  while (1)
  {
    if (Ve == 1)
      Ve = 0;
    else
      Ve = 1;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.printf("Bonjour \n\r");
 
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      10,         // tres haut niveau de priorite
      NULL        // descripteur
  );
  xTaskCreate(
      Vin,   // nom de la fonction
      "Vin", // nom de la tache que nous venons de vréer
      10000, // taille de la pile en octet
      NULL,  // parametre
      1,     // bas niveau de priorite
      NULL   // descripteur
  );

  // calcul coeff filtre
  Serial.begin(115200);
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;
}

void reception(char ch)
{
  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }

    if (commande == "Te")
    {
      Te = valeur.toInt();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void loop()
{
  if (FlagCalcul == 1)
  {
    Serial.printf("%f %f %f %f\n",TetaG,TetaGF,TetaWF,angle); // Affichage des angles sur le moniteur série

    FlagCalcul = 0;
  }
}
void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}