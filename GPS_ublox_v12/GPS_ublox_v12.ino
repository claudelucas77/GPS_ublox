
// Librairies
#include <HardwareSerial.h>     // Pour com GPS
#include <TFT_eSPI.h>           // Pour afficheur
#include <SPI.h>                // Pour afficheur en com SPI
#include <Wire.h>               // Pour SPI
// #include "bmp.h"                // Image d'acceuil, ne sert à que dalle

#include "BluetoothSerial.h"    // Com Bluetooth avec telephone
#include "FS.h"                 // Pour data log sur la flash
#include <LittleFS.h>           // Pareil
#include "esp_bt.h"             // Librairie Bluetooth qui permet de le couper pour eviter les perturbations


#include <SD.h>                 // Pour datalogging (à retirer car plus de carte SD)

// Pins de l'afficheur
#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34


// Ecran
TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

// GPS
HardwareSerial gpsSerial(2); // UART2 sur GPIO16(RX)/GPIO17(TX)



// Variables


char buff[512];
int vref = 1100;
float battery_voltage = 0;
bool BToff = 0;
bool DebugBT = 0;
bool Debug = 0;

// Variables GPS

uint16_t annee  = 0;
uint8_t mois  = 0;
uint8_t jour    = 0;
uint8_t heure   = 0;
uint8_t minute    = 0;
uint8_t seconde    = 0;
uint32_t nanosec  = 0;
uint32_t iTOW = 0;

// Position
double lat = 0;
double lon = 0;
double hMSL = 0;

// Vitesse
double velN  = 0;
double velE  = 0;
double velD  = 0;
double gSpeed = 0;
double heading = 0;

// Accuracies
double hAcc = 0;
double vAcc = 0;
double sAcc = 0;
double HeadAcc = 0;

int fixType = 0;
int numSV   = 0;

// Variable qui contient la ligne à ajouter
String ligne;
char timestr[32];

// Variables horloge
bool fm500ms = 0;    /// Front montant 500ms
bool fm200ms = 0;    /// Front montant 200ms
bool fm30s = 0;      /// Front montant 30s
bool cligno = 0;     /// Bit clignotant 500ms
bool Dmd_Flush = 0;  /// Demande flush (Ecriture fichier SD)


// Variables pour le fichier (Bibliotheque Flash LittleFS)
bool partition_nok = 0;   /// Problème de partition Flash (besoin de formater par exemple) : Voir File/example/LittleFS/LittleFS Test
bool REC_SD = 0;          /// Enregistrement en cours
bool FM1 = 0;             /// Permet de n'effectuer qu'une fois la mise en REC_SD
bool fichier_defini = 0;  /// Nom de fichier defini
File dataFile;            /// Variable du Fichier
String nom_fichier = "";  /// Variable du nom de fichier
#define FORMAT_LittleFS_IF_FAILED true

// Variables pour la com avec la console
char reply[50];     /// Variable Réception du port Sérial1 (PC)
char reply_BT[50];  /// Variable Réception du port Sérial1 (BLUETOOTH)

bool GPS_OFF = 0;  /// Désactive la lecture GPS


//  Variables Bluetooth
BluetoothSerial SerialBT;  /// Déclaration variable pour Bluetooth




// Trames de commande pour la puce GPS

// NAV-PVT ON
const uint8_t CFG_MSG_NAV_PVT[] = {
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,
  0x01, 0x07, 0x01,   // Class=0x01(NAV), ID=0x07(PVT), Rate=1 sur ce port
  0x13, 0x51
};

// RATE 5Hz
const uint8_t CFG_RATE_5HZ[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xC8, 0x00,   // measRate=200ms -> 5Hz
  0x01, 0x00,   // navRate=1
  0x01, 0x00,   // timeRef=UTC
  0xDE, 0x6A
};

// Disable NMEA
const uint8_t CFG_MSG_NMEA_OFF[][11] = {
  {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F}, // GGA
  {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11}, // GLL
  {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13}, // GSA
  {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15}, // GSV
  {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17}, // RMC
  {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19}  // VTG
};

// Réglage sur Dynamic mode = 6 (<1G) (Typique ublox, c'est pour le lissage des vitesses)
const uint8_t setDynModel1G[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
    0xFF, 0xFF,  // mask: 0xFFFF (tous les paramètres)
    0x06,        // dynModel: Airborne <1G
    0x03,        // fixMode: Auto 2D/3D
    0x00, 0x00, 0x00, 0x00,  // fixedAlt
    0x10, 0x27, 0x00, 0x00,  // fixedAltVar
    0x05,        // minElev: 5°
    0x00,        // drLimit
    0xFA, 0x00,  // pDop: 25.0
    0xFA, 0x00,  // tDop: 25.0
    0x64, 0x00,  // pAcc: 100m
    0x2C, 0x01,  // tAcc: 300m
    0x00,        // staticHoldThresh
    0x3C,        // dgnssTimeout: 60s
    0x00, 0x00,  // cnoThresh
    0x00, 0x00,  // reserved
    0xC8, 0x00,  // staticHoldMaxDist: 200m
    0x00,        // utcStandard
    0x00, 0x00, 0x00, 0x00, 0x00,  // reserved
    0x1A, 0x28   // Checksum (à recalculer)
};



// Fonction envoi commande au GPS
void sendUBX(const uint8_t* msg, size_t size) {
  gpsSerial.write(msg, size);
  delay(150); // délai pour que le module traite la commande
}

// Fonction calcul Tension pour Etat charge batterie
void CalculVoltage()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        
    }
}

void setup() {

  // Init pin pour afficheur
    pinMode(ADC_EN, OUTPUT);
    digitalWrite(ADC_EN, HIGH);

// Init couleurs + taille etc de l'afficheur
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(4);
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(0, 0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(4);
    afficher("GPS Logger", 3, 120, 70);

// image d'acceuil
  //  tft.setSwapBytes(true);
  //  tft.pushImage(0, 0,  240, 135, ttgo);
  //  delay(2000);
  //  tft.fillScreen(TFT_BLACK);

// Pour diagnostic pendant developpement
  Serial.begin(2000000);

// Init GPS
  gpsSerial.begin(9600, SERIAL_8N1, 38, 17); // RX, TX pour UART2
  delay(1000);

  Serial.println("Configuring u-blox GPS...");


// Envoi des commandes au GPS pour config
  // Désactiver NMEA
  for (int i = 0; i < 6; i++) sendUBX(CFG_MSG_NMEA_OFF[i], sizeof(CFG_MSG_NMEA_OFF[i]));

  // Activer NAV-PVT
  sendUBX(CFG_MSG_NAV_PVT, sizeof(CFG_MSG_NAV_PVT));

  // Régler la fréquence à 5Hz
  sendUBX(CFG_RATE_5HZ, sizeof(CFG_RATE_5HZ));

  // Régler Dynamic mode sur 6 (lissage modéré conseillé par le gars de Flysight)
  sendUBX(setDynModel1G, sizeof(setDynModel1G));


  Serial.println("Config done. Forwarding all GPS data...");


  if (!LittleFS.begin(FORMAT_LittleFS_IF_FAILED)) {
    partition_nok = true;
    Serial.println("partition pas créée");
  }


  /// Bluetooth
  SerialBT.begin("Logger GPS Claudius");  /// Liaison série Bluetooth


}




void loop() {

  CalculVoltage();

  affichage();
 
  GPS();

  horloge();

  commandes_console();

  serial_console(); 


  stop_bluetooth();


// Flush l'enregistrement toutes les 30s
  if (fm30s) {

    int time1 = millis();
    dataFile.flush();
  //  Serial.print("Flush : ");
  //  Serial.println(millis() - time1);
  }





}

// Fonction qui récupere les trames GPS et qui teste si une trame est complète
void GPS() {
  static uint8_t buf[256];  // buffer pour stocker la trame UBX en cours de lecture
  static int idx = 0;        // index courant dans le buffer

  // Tant qu'il y a des octets disponibles sur le port GPS
  while (gpsSerial.available()) {
    uint8_t b = gpsSerial.read();  // lire un octet

    // Détection du début de trame UBX (0xB5 0x62)
    if (idx == 0 && b != 0xB5) continue; // ignorer jusqu'à 0xB5
    buf[idx++] = b;  // stocker l'octet dans le buffer

    // Vérifier le deuxième octet de sync
    if (idx == 2 && buf[1] != 0x62) { 
      idx = 0;       // si ce n'est pas 0x62, réinitialiser l'index et attendre un nouveau début
      continue; 
    }

    // Dès qu'on a lu l'entête complète (6 octets), on peut connaître la longueur de la payload
    if (idx >= 6) {
      int len = buf[4] | (buf[5] << 8);  // longueur de la payload = bytes 4 et 5
      // Vérifier si la trame complète (header + payload + checksum) est reçue
      if (idx == len + 8) {  // 8 = 2 octets sync + 2 class/id + 2 length + 2 checksum

      // Appel de la fonction qui récupère les données dans la trame
        parseNAVPVT(buf, len + 8);  // parser la trame si c'est NAV-PVT

        archivage(); // lance l'archivage
        idx = 0;  // réinitialiser le buffer pour la prochaine trame      }
      } 
    }
  }
}

// Fonction qui récupère les données dans la trame
void parseNAVPVT(uint8_t* buf, int len) {
  if (buf[2] != 0x01 || buf[3] != 0x07) return; // pas NAV-PVT

  uint8_t* p = buf + 6;

  // UTC date/time
  annee     = p[4] | (p[5] << 8);
  mois      = p[6];
  jour      = p[7];
  heure     = p[8];
  minute   = p[9];
  seconde  = p[10];
  nanosec   = p[16] | (p[17]<<8) | (p[18]<<16) | (p[19]<<24);
  nanosec   = nanosec / 10000000;
  iTOW      = p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24); 
  iTOW      = (iTOW % 1000) / 10;   // Modulo pour prendre les 3 derniers chiffres 

  //Serial.print(p[16]);Serial.print("-");Serial.print(p[17]);Serial.print("-");Serial.print(p[18]);Serial.print("-");Serial.print(p[19]);Serial.print("-");Serial.print(nanosec);Serial.print("-");
  //Serial.println(iTOW);


  // Position
  lat = (*(int32_t*)&p[28]) / 1e7; // 1e-7 deg
  lon = (*(int32_t*)&p[24]) / 1e7;

  hMSL = *(int32_t*)&p[36]; // mm
  hMSL = hMSL / 1000; //m

  // Vitesse
  velN  = *(int32_t*)&p[48];
  velN  = velN / 1000; // conversion m
  
  velE  = *(int32_t*)&p[52];
  velE  = velE / 1000 ; // conversion m

  velD  = *(int32_t*)&p[56];
  velD = velD / 1000; // conversion m

  gSpeed = *(uint32_t*)&p[60];        // ground speed (Vit horizontale) ne sert pas

  heading = *(int32_t*)&p[64];
  heading = heading / 100000;

  // Accuracies
  hAcc = *(uint32_t*)&p[40];
  hAcc = hAcc / 1000; // mm

  vAcc = *(uint32_t*)&p[44];
  vAcc = vAcc / 1000;

  sAcc = *(uint32_t*)&p[68];
  sAcc = sAcc / 1000;

  HeadAcc = *(uint32_t*)&p[72];
  HeadAcc =   HeadAcc / 100000;

  fixType = p[20];
  numSV   = p[23];

  // Format timestamp ISO 8601
  
  snprintf(timestr, sizeof(timestr), "%04d-%02d-%02dT%02d:%02d:%02d.%02dZ",
           annee, mois, jour, heure, minute, seconde, iTOW);


  
  definir_ligne();

}


void horloge() {

  static int tps_500ms;
  static int tps_200ms;
  static long tps_30s;


  fm500ms = 0;
  fm200ms = 0;
  fm30s = 0;

  if (millis() > tps_500ms + 500) {
    tps_500ms = millis();
    fm500ms = 1;
    cligno = !cligno;
  }
  if (millis() > tps_200ms + 200) {
    tps_200ms = millis();
    fm200ms = 1;
  }

  if (millis() > tps_30s + 30000) {
    tps_30s = millis();
    fm30s = 1;
  }

  if (fm30s) {

    Dmd_Flush = 1;
  }
}

void affichage() {

  // Afficheur 240x135 (x,y)
  if (fm500ms) {

    // Refresh ecran
    tft.fillScreen(TFT_BLACK);
    String voltage = String(battery_voltage) + "V";

    // Charge batterie
    afficher(voltage, 2, 0, 0);

    // Altitude
    afficher(String(hMSL,0), 3, 110, 70);
    afficher("m", 2, 170, 75);
    

    /// Nb de satellites
    afficher("sat", 2, 140, 100);
    afficher(String(numSV), 2, 190, 100);  /// Vers afficheur

    /// Partition OK
    if (!partition_nok) {
      afficher("SD OK", 2, 190, 0);  /// Affiche SD si la sd est ok
    } else {
      afficher("SD NOK", 2, 190, 0);
    }

    if (!BToff) {
      afficher("Bt", 2, 120, 0);  /// Affiche SD si la sd est ok

    }

    // notif enregistrement qui clignote
    if (REC_SD && cligno) afficher("o", 2, 10, 115);  /// Si enregistrement en cours le o clignote


    // Nom fichier
    if (fichier_defini) {  /// Affichage nom du fichier en cours d'écriture
      afficher(nom_fichier.c_str(), 2, 150, 35);
    }
  }


    
}



// Affiche du texte sur l'afficheur 
void afficher(String texte, int taille, int x,int y) {
  tft.setTextSize(taille);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(texte,  x, y );
}


void definir_ligne() {

  ligne = "";
  ligne += timestr;
  ligne += (F(","));
  ligne += (String(lat, 7));
  ligne += (F(","));
  ligne += (String(lon, 7));
  ligne += (F(","));
  ligne += (String(hMSL, 3));
  ligne += (F(","));
  ligne += (String(velN, 2));
  ligne += (F(","));
  ligne += (String(velE, 2));
  ligne += (F(","));
  ligne += (String(velD, 2));
  ligne += (F(","));
  ligne += (String(hAcc, 3));
  ligne += (F(","));
  ligne += (String(vAcc,3));
  ligne += (F(","));
  ligne += (String(sAcc,2));
  ligne += (F(","));
  ligne += (String(heading,5));
  ligne += (F(","));
  ligne += (String(HeadAcc,5));
  ligne += (F(","));
  ligne += (String(fixType));
  ligne += (F(","));
  ligne += (String(numSV));

 if (Debug) {
  Serial.println(ligne);
 }
 if (DebugBT) {
  SerialBT.println(ligne);
 }



}


void archivage() {


  if (REC_SD) {

    /// Si début d'enregistrement, on définit un nom de fichier libre
    if (!fichier_defini) {

      nom_fichier = "/";
      nom_fichier += (format(jour));
      nom_fichier += (format(mois));
      nom_fichier += (F("_"));
      nom_fichier += (format(heure));
      nom_fichier += (format(minute));
      nom_fichier += (format(seconde));

      fichier_defini = true;




      dataFile = LittleFS.open(nom_fichier.c_str(), FILE_APPEND);  /// Ouverture fichier

      dataFile.println(F("time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,heading,cAcc,gpsFix,numSV"));
      dataFile.println(F(",(deg),(deg),(m),(m/s),(m/s),(m/s),(m),(m),(m/s),(deg),(deg),,"));


    }  /// Fin création nouveau fichier





    /// Concaténation de la ligne à remplir dans le fichier

    if (dataFile) {

      dataFile.println(ligne);
    }
  }

}  /// Fin archivage()




void commandes_console() {

  //// Appui BP REC

  if (hMSL != 0 && numSV >5 && battery_voltage < 4.5 && !FM1) {

    REC_SD = true;
    FM1 = 1;
  }



  /// LECTURE SERIAL
  if (test_char(reply, "lire ", 5)) {

    REC_SD = false;
    String Cible_lecture = "";
    int i = 5;

    Cible_lecture += '/';
    while (reply[i] != '\0') {

      Cible_lecture += reply[i];
      i++;
    }

    // Remplacement des caractères de retour à la ligne si présents
    Cible_lecture.replace("\n", "");
    Cible_lecture.replace("\r", "");

    dataFile = LittleFS.open(Cible_lecture.c_str());
    if (dataFile) {
      while (dataFile.available()) {

        Serial.write(dataFile.read());
      }
      Serial.println("%");
    }
  }

  /// LECTURE BLUETOOTH
  if (test_char(reply_BT, "lire ", 5)) {

    REC_SD = false;
    String Cible_lecture = "";
    int i = 5;

    Cible_lecture += '/';
    while (reply_BT[i] != '\0') {

      Cible_lecture += reply_BT[i];
      i++;
    }

    // Remplacement des caractères de retour à la ligne si présents
    Cible_lecture.replace("\n", "");
    Cible_lecture.replace("\r", "");

    dataFile = LittleFS.open(Cible_lecture.c_str());
    if (dataFile) {
      int i = 0;
      while (dataFile.available()) {

        SerialBT.write(dataFile.read());
      }
      SerialBT.println("%");
    }
  }


  /// Test ecriture fichier
  if (test_char(reply, "test", 4)) {
    REC_SD = false;
  }



  if (test_char(reply, "dir", 3)) {

    REC_SD = false;
    listDir(LittleFS, "/", 3);
  }

  if (test_char(reply_BT, "dir", 3)) {
    REC_SD = false;
    listDirBT(LittleFS, "/", 3);
  }

  if (test_char(reply, "log", 3)) {

    REC_SD = false;
    Debug = true;
  }

  if (test_char(reply_BT, "log", 3)) {

    REC_SD = false;
    DebugBT = true;
  }



  if (test_char(reply_BT, "format", 6)) {

    REC_SD = false;
    format_mem(LittleFS, "/");
  }

  if (test_char(reply, "format", 6)) {

    REC_SD = false;
    format_mem(LittleFS, "/");
  }



  /// DELETE SERIAL
  if (test_char(reply, "del ", 4)) {

    REC_SD = false;
    String Cible_lecture = "";
    int i = 4;

    Cible_lecture += '/';
    while (reply[i] != '\0') {

      Cible_lecture += reply[i];
      i++;
    }

    // Remplacement des caractères de retour à la ligne si présents
    Cible_lecture.replace("\n", "");
    Cible_lecture.replace("\r", "");

    LittleFS.remove(Cible_lecture);
  }
}

void serial_console() {


  ///// Réception Serial /////
  int j = 0;
  while (Serial.available()) {
    reply[j] = Serial.read();
    j += 1;
  }
  reply[j] = '\0';

  if (reply[0] != 0) {
    Serial.print("Message de Serial : ");
    Serial.println(reply);
  }


  ////// Réception Bluetooth
  int i = 0;

  while (SerialBT.available()) {
    reply_BT[i] = SerialBT.read();
    i += 1;
  }

  reply_BT[i] = '\0';

  if (reply_BT[0] != '\0') {
    Serial.print("Message de BT : ");
    Serial.println(reply_BT);
    GPS_OFF = 1;
  }
}

String format(byte nb) {
  String chaine = "";

  if (nb < 10) {
    chaine += "0";
  }
  chaine += nb;

  return (chaine);
}


//// test des chaines de caractères sur xxx caractères
bool test_char(char chaine1[], char chaine2[], int nb_car) {

  bool result = true;

  for (int i = 0; i < nb_car; i++) {

    if (chaine1[i] != chaine2[i]) {
      result = false;
    }
  }

  return result;
}






//////////////////////////////////////////////////////////////:
//////         Bibliotheque LittleFS                ////////////
//////////////////////////////////////////////////////////////:

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}




// void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
//   Serial.printf("Listing directory: %s\r\n", dirname);

//   File root = fs.open(dirname);
//   if (!root) {
//     Serial.println("- failed to open directory");
//     return;
//   }

//   File file = root.openNextFile();
//   while (file) {
//       Serial.print(file.name());
//       Serial.print("\tSIZE: ");
//       Serial.println(file.size());

//     file = root.openNextFile();
//   }
// }
void listDirBT(fs::FS &fs, const char *dirname, uint8_t levels) {
  //SerialBT.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    //SerialBT.println("- failed to open directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {

    SerialBT.print(file.name());
    SerialBT.print("\t");
    SerialBT.print(file.size());
    SerialBT.println(" bytes");

    file = root.openNextFile();
  }
  SerialBT.print("!");  // Fin de message
}



void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}





void format_mem(fs::FS &fs, const char *dirname) {  ////////////// Efface tous les fichiers sauf le dernier (je ne sais pas pourquoi il ne veut pas)
  Serial.printf("Format");
  String chemin;


  File root = fs.open(dirname);


  File file = root.openNextFile();

  while (file) {

    chemin = dirname;
    chemin += file.name();
    file.close();

    fs.remove(chemin);

    file = root.openNextFile();
  }
}

void stop_bluetooth() {

  if (millis() > 30000 && !SerialBT.hasClient()) {
      SerialBT.end();
      btStop();                // libère complètement la radio BT
      BToff = true;

  }


}