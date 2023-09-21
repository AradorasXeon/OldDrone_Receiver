
//Antenna dolgai - RadioRecive_0005_irg vagy hasonló nevű teszt progi az alap
//A 8-as nagyon faszán működik, itt némi átiratt, hogy a i2c kommunikáció jobban menjen
//A 9-es is nagyon jó, ez már inkább stilisztika
//11-es verzió, GPS fizikailag lekerül a drónról, ill. a cella feszültség is máshol lesz figyelve, ezen részek eltávolítása
//12-es verzió: 11-esbe csak kikommentelve a dolgok, itt ki is törlöm, hogy átlátható legyen
//13-as verzió: Lévén, hogy ennyi mindent kiszedtem belőle, most tesztek alapján úgy tűnik, jobb lenne, ha i2c-n rögtön csak az menne ki
//, hogy melyik motor mennyivel menjen (v12 22% prog tárhely, 23% dinMem, ez várhatóan nő, meg minimális lassulást okoz majd a program futásban, de a SLAVE
// -nek több ideje lesz elenőrizni az aksi cellákat
//14 köztes tisztázás
//15 leírások tisztázása - video ready kommentelések

//NRF24L01
#include <RF24.h>
//********************************************************
//MPU6050
#include <Wire.h>
//********************************************************

//****************************************************************************************
//**********************************Konstansok******************************************
//****************************************************************************************

//========================================================
//Antenna változók/konstansok
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001"; //rádió címe
struct Data_Package //Data "formátum", amiben a távirányító küldi a dolgokat
{
  byte passKEY;
  byte j1_x;
  byte j1_y;
  byte j1_b;
  byte j2_x;
  byte j2_y;
  byte j2_b;
  byte empty;
};
//Megmondjuk a programnak, hogy ezek változó dolgok, ne akarjon okoskodni a fordító
volatile Data_Package data;
volatile bool newData = false; //Interrupthoz, hogy új adat-e
const uint16_t MAXRADIOIDLETIME = 180; //kb 60 ~= 1-2 sec, ha nem kap jelet csináljon valamit
uint16_t radioIdleCounter = 0;
bool emergencyStopRad; //megszűnt rádió vétel
bool firstSignalArrived = false;
//********************************************************
static const uint16_t Baud = 9600;
//********************************************************
//Accel/Gyro konstansok, változók
volatile float prevTime = 0;
volatile float actualTime = 0;
volatile float timePassed = 0;
int16_t mems[6]; //nekünk ugye plusz minusz kell
const float DIVIDER_GYRO = 16.4; //doksi szerinti adat beírva // lenne: 16.3835; // int_16t range of -32,768 to 32,767    tehát 32767-et ír ki 2000 deg/s-nál
const float DIVIDER_ACCEL = 2048; //2047.9375; //+ - 16 g  ha m/s^2-ben akarjuk megkapni, akk meg kell szorozni 9,81-gyel


float angles[3];
float floatMems[6];
float errors[6];
float errorsSUM[6];

//********************************************************

//Sebesség vezérlés

byte motoSpeed[4];
byte motoThrust[4];
byte motoX[4];
byte motoY[4];

const byte FOR_BACK_SWITCH_NUM = 25; //a thrust kar alapból középre húz vissza, ezen a környékén a thrust legyen az itt megadott szám
const byte MOTO_XY_MAX_NUM = 20;
const byte MAX_MOTO_THRUST = 120; //ehez még hozzáadódhat a moto xy !!!!!
const float MAX_ANGLE = 15; //max elfordulás, x és y tengely körül fokban megadva, azért float, mert nem tudom jó ötlet-e byte-ként megadni egy float-os összehasonlításnál

//****************************************************************************************
//**********************************SETUP*************************************************
//****************************************************************************************

void setup() 
{
  SafetyZeroing(); //biztonsági okokból
  Serial.begin(Baud); 
  // Rádió alap beállításai:
TRYAGAIN:
  if (!radio.begin()) 
  {
    Serial.println(F("Radio hardware not responding!"));
    delay(125);
    goto TRYAGAIN;
  }
  radio.openReadingPipe(1, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN); 
  radio.setDataRate(RF24_1MBPS); //250 kbs bőven elég lenne, de azt csak a + tudja elvileg (hardware)
  radio.setChannel(77);
  radio.setPayloadSize(8); // 7 db van, de legyen szép szám
  radio.startListening();              //This sets the module as receiver
  Serial.println(F("Attaching interrupt!"));
  attachInterrupt(digitalPinToInterrupt(2), ISR_icomingSignal, FALLING);
  Serial.println(F("Antenna Ready!"));
  delay(10);
  //********************************************************
  
  //MPU6050
  Wire.begin(); 
  MPU6050_Write(0x6B, 0b10000000); //Reset-eli a MEMS chipet
  delay(100);
  Awaken(); //Felébreszti a MEMS chipet, ha kell, bár elvileg a reset ezt kiüti, biztos, ami sicher....
  //Regiszterekbe beállítások átirkálása
  MPU6050_Write(0x1B, 0b00011000); //± 2000 °/s - max tartomány
  MPU6050_Write(0x1C, 0b00011000); //± 16g      - max tartomány
  MPU6050_Write(0x19, 0b00000000); //mintavételezés gyakorisága (legsűrűbb)
  MPU6050_Write(0x1A, 0b00000000); //8 kHz helyett 1 kHz-re lehet állítani, a 0 a 8 kHz-nek felel meg
  //********************************************************
  //Kommunikáció a másik NANO-val
  //Ahhoz se kell ide semmi

  prevTime = millis(); //ne teljesen hülyeséget számoljon elfordulásnak az elején
  
  //MEMS hiba begyűjtése:
  for(uint16_t i = 0; i<500; i++)
  {
    GetAccelGyroData();
    for(byte j = 0; j<6; j++)
    {
      errorsSUM[j] += floatMems[j];
    }
  }
  
    for(byte j = 0; j<6; j++)
    {
      errors[j] = errorsSUM[j] / 500;
    }

  Serial.println("Errors:");
  
  errors[0] = 0 - errors[0];
  Serial.println(errors[0]);

  errors[1] = 0 - errors[1];
  Serial.println(errors[1]);

  errors[2] = 1 - errors[2]; //ez a z tengely, itt 1-et kéne mutatnia, csak ezért nem írom for loop-ba, bár biztos szebb lenne egy if-et belenyomni
  Serial.println(errors[2]);

  errors[3] = 0 - errors[3];
  Serial.println(errors[3]);

  errors[4] = 0 - errors[4];
  Serial.println(errors[4]);

  errors[5] = 0 - errors[5];
  Serial.println(errors[5]);
  
  Serial.println(F("MPU Ready!"));
  
}

//****************************************************************************************
//**********************************LOOP**************************************************
//****************************************************************************************
void loop()
{

  //PRIO 1: Rádióból jövő adatok
  ELEJE:
  if (data.passKEY != 88 && newData)
  {
    Serial.println(F("Not good pass key. Someoneelse is probably using same channel."));
    newData = false;
  }
        
  if (newData)
    {
      //Reading the data
      firstSignalArrived = true;
      radioIdleCounter = 0; //nullázuk a számlálót
      emergencyStopRad = false; //ha visszajön a jel
      newData = false;     
    }
    else //ha nincs új adat
    {  
      radioIdleCounter++;
      SafetyZeroing();
      if (radioIdleCounter > MAXRADIOIDLETIME)
      {
        emergencyStopRad = true;
      }
    }

    //Végeztünk az antenna kód részével
    //****************************************************************************************************************************
    //****************************************************************************************************************************

    GetAccelGyroData();
    IfTooSmall(); //ha 0,5 °/s -alatt van, akkor az nulla
    
    // Itt kell eldönteni, hogy melyik motor milyen sebességgel menjen

    /*
   * Motor 4      ^Y^         Motor 1
   *               I
   *               I
   * ================================>>>>>>> X TENGELY
   *               I
   *               I
   * Motor 3       I          Motor 2
   * 
    */

  //******************** Irányzék vezérlés

  //Ha nullán áll a kar, akkor próbáljon meg vízszintbe kerülni.

// default data value:
/*
data.j1_x = 127;
data.j1_y = 127;
data.j1_b = 0;
data.j2_x = 127;
data.j2_y = 127;
data.j2_b = 0;
*/

// motoThrust[4];     0,6*255  154 egyelőre
// motoX[4] és motoY;  0,2*255  51 --> 25 + 25 --ez valszeg függeni függ a dőlésszögesditől 
//Minden maximum 80%-on pöröghet TESZTELNI

//bal joy Y - sebesség
//jobb joy Y - drone X - axis előre (aksi pozíció, jelenlegi összeszerelés alapján: Eleje az aksi drót nélküli oldala az aksi hossz tengely mentén)
//jobb joy X - drone Y (balra Y pozitív irány) 

//const byte FOR_BACK_SWITCH_NUM = 25; //a thrust kar alapból középre húz vissza, ezen a környékén a thrust legyen az itt megadott szám
//const byte MOTO_XY_MAX_NUM = 20;
//const byte MAX_MOTO_THRUST = 120; //ehez még hozzáadódhat a moto xy !!!!!
  
  //Ha döntve van a kar, akkor megfelelő irányú motorok-ra adjon pluszt, de max 15-30°-ot dőljön a vízszinthez képest
if(firstSignalArrived)
{
  if (data.j2_y > 127) //X hátra
  {
    motoX[0] = map(data.j2_y, 128, 255, 0 , MOTO_XY_MAX_NUM) ; //1-es
    motoX[1] = map(data.j2_y, 128, 255, 0 , MOTO_XY_MAX_NUM) ; //2-es
    motoX[2] = 0; //3-as
    motoX[3] = 0; //4-es
  }
  else //X előre ELLENŐRIZD, HOGY FIZIKAILAG AZ TÉNYLEG Y IRÁNY-E
  {
    motoX[0] = 0; //1-es
    motoX[1] = 0; //2-es
    motoX[2] = map(data.j2_y, 127, 0, 0 , MOTO_XY_MAX_NUM); //3-as
    motoX[3] = map(data.j2_y, 127, 0, 0 , MOTO_XY_MAX_NUM); //4-es
  }

  if (data.j2_x > 127)
  {  
    motoY[0] = map(data.j2_x, 128, 255, 0 , MOTO_XY_MAX_NUM); //1-es
    motoY[1] = 0; //2-es
    motoY[2] = 0; //3-as
    motoY[3] = map(data.j2_x, 128, 255, 0 , MOTO_XY_MAX_NUM); //4-es
  }
  else
  {
    motoY[0] = 0; //1-es
    motoY[1] = map(data.j2_x, 127, 0, 0 , MOTO_XY_MAX_NUM); //2-es
    motoY[2] = map(data.j2_x, 127, 0, 0 , MOTO_XY_MAX_NUM); //3-as
    motoY[3] = 0; //4-es
  }

  //******************** Sebesség vezérlés
  // 0-135-ig a motor 20-30%-ka, ami épp felemeli TESZTELNI KELL

  if (data.j1_y > 135) //EMELKEDIK 0,6*255  154 egyelőre
  {
    motoThrust[0] = map(data.j1_y, 136, 255, FOR_BACK_SWITCH_NUM , MAX_MOTO_THRUST); //1-es
    motoThrust[1] = map(data.j1_y, 136, 255, FOR_BACK_SWITCH_NUM , MAX_MOTO_THRUST); //2-es
    motoThrust[2] = map(data.j1_y, 136, 255, FOR_BACK_SWITCH_NUM , MAX_MOTO_THRUST); //3-as
    motoThrust[3] = map(data.j1_y, 136, 255, FOR_BACK_SWITCH_NUM , MAX_MOTO_THRUST); //4-es
  }
  else //LEBEG - süllyed
  {
    motoThrust[0] = map(data.j1_y, 0, 135, 0 , FOR_BACK_SWITCH_NUM); //1-es
    motoThrust[1] = map(data.j1_y, 0, 135, 0 , FOR_BACK_SWITCH_NUM); //2-es
    motoThrust[2] = map(data.j1_y, 0, 135, 0 , FOR_BACK_SWITCH_NUM); //3-as
    motoThrust[3] = map(data.j1_y, 0, 135, 0 , FOR_BACK_SWITCH_NUM); //4-es
  }
}

//szögek ellenörzése angles[0] - elv x tengely körül, [1]-es y tengely körül

if(angles[0] > MAX_ANGLE)
{ //1 és 4-es lassuljon
    motoX[0] = 0; //1-es
    motoX[3] = 0; //4-es
}

if(angles[0] < -1*MAX_ANGLE)
{
  //2 és 3-as lassuljon
    motoX[1] = 0; //2-es
    motoX[2] = 0; //3-as
}

//elv Y körüli elfordulás
if(angles[1] > MAX_ANGLE)
{
  //3 és 4-es lassuljon
    motoY[2] = 0; //3-as
    motoY[3] = 0; //4-es
}

if(angles[1] < -1*MAX_ANGLE)
{
  //1 és 2-as lassuljon
    motoY[0] = 0; //1-es
    motoY[1] = 0; //2-es
}


  //FINALLY:
    for (byte i = 0; i<4; i++)
    {
      motoSpeed[i] = motoThrust[i] + motoX[i] + motoY[i];
      
    }
    
    if(emergencyStopRad)
    {
      for (byte i = 0; i<4; i++)
      {
        motoSpeed[i] = 0;
      }
    }

    //Kiírás teszt okokból
    for (byte i = 0; i<4; i++)
    {
      Serial.print(motoSpeed[i]);
      Serial.print("\t");
    }
    Serial.println();

    //****************************************************************************************************************************
    //****************************************************************************************************************************
    //Adatok kiírása i2c-re
      Wire.beginTransmission(0xF2); //mems chip mindig lezárásra kerül
      Wire.write(motoSpeed, 4);
      Wire.endTransmission(true); //max 32 bájtot bír átküldeni egy adagban
      delay(2);
//****************************************************************************************************************************
    //Serial.println();
    
//******************************************** END OF LOOP ***********************************************
//********************************************************************************************************
//********************************************************************************************************
//********************************************************************************************************
//********************************************************************************************************
}





//****************************************************************************************
//**********************************Szub rutinok******************************************
//****************************************************************************************

//Rádióhoz
void ISR_icomingSignal() //Interrupt szubrutin
{
  radio.read(&data, radio.getPayloadSize());
  newData = true; //Megmondjuk, hogy új adatok vannak eltárolva az antennáról a data nevű változóban
}

//****************************************************************************************
//MPU6050-hez
int16_t PrintRegData(int Hadress, int Ladress)
{
  byte H;
  byte L;
  MPU6050_Read(Hadress, &H);  // Get data
  MPU6050_Read(Ladress, &L);  // Get data
  int16_t data = H << 8 | L; //azért nem unsigned, mert -/+ módban üzemel!
  
  //Serial.print(data);
  //Serial.print("\t");
  return data;
}

void Awaken()
{
  uint8_t awakeByte;                                 // Data will go here
  MPU6050_Read(0x6B, &awakeByte);      // Get data
  boolean sleepModeOn = bitRead(awakeByte,6);        // Check if sleep mode on
  while(sleepModeOn)
  {                                // If on...
    delay(500);                                      // delay
    bitClear(awakeByte,6);                           // Clear bit of sleep
    MPU6050_Write(0x6B, 0b00000000);   // Send data back
    delay(500);                                      // delay
    MPU6050_Read(0x6B, &awakeByte);    // Get data again
    sleepModeOn = bitRead(awakeByte,6);              // If sleep mode off, exit loop
  }                                                  //
  delay(500);                                        // delay
}

void MPU6050_Read(int address,uint8_t *data)
{            // Read from MPU6050. Needs register address, data array
  int size = sizeof(*data);                              //
  Wire.beginTransmission(0x68);           // Begin talking to MPU6050
  Wire.write(address);                                   // Set register address
  Wire.endTransmission(false);                           // Hold the I2C-bus
  Wire.requestFrom(0x68, size, true);     // Request bytes, release I2C-bus after data read
  int i = 0;                                             //
  while(Wire.available())
  {                               //
    data[i++]=Wire.read();                               // Add data to array
  }
  Wire.endTransmission();
}

void MPU6050_Write(int address,byte dbytes) // Write to MPU6050. Needs register address, data array
{  
  Wire.beginTransmission(0x68);    // Begin talking to MPU6050
  Wire.write(address);                            // Set register address
  Wire.write(dbytes);                        // akkor csak byte-onként küldünk adatot
  Wire.endTransmission();                         // Release I2C-bus
}


void GetAccelGyroData()
{
    //Gyorsulás és Gyro adatok beolvasása
    actualTime = millis();
    // g-ben méri a gyorsulást; °/s-ban a szögelfordulást 

    //a high és low byte összekötése
    mems[0] = PrintRegData(0x3B, 0x3C);
    mems[1] = PrintRegData(0x3D, 0x3E);
    mems[2] = PrintRegData(0x3F, 0x40);
    mems[3] = PrintRegData(0x43, 0x44);
    mems[4] = PrintRegData(0x45, 0x46);
    mems[5] = PrintRegData(0x47, 0x48);
    
    timePassed = (actualTime - prevTime) / 1000;

    floatMems[0] = mems[0] / DIVIDER_ACCEL;
    floatMems[1] = mems[1] / DIVIDER_ACCEL;
    floatMems[2] = mems[2] / DIVIDER_ACCEL;
    
    floatMems[3] = mems[3] / DIVIDER_GYRO;
    floatMems[4] = mems[4] / DIVIDER_GYRO;
    floatMems[5] = mems[5] / DIVIDER_GYRO;

    for(byte i = 0; i<6; i++)
    {
      floatMems[i] += errors[i]; //az error ugye kalibrálásnál még nulla
    }
    
    prevTime = actualTime;
}


void IfTooSmall()
{
  for(byte i = 3; i<6; i++)
    {
      floatMems[i] = (floatMems[i] < 0.5 && floatMems[i] > -0.5 ? 0.0 : floatMems[i]); //az error ugye kalibrálásnál még nulla
    }
    //0,5 plusz mínuszba még error után is hajlamos volt "beképzelni", ezért inkább az az alati jeleket levágjuk

    //timePassed direkt marad az adat beolvasásnál
    angles[0] += floatMems[3] * timePassed;
    angles[1] += floatMems[4] * timePassed;
    angles[2] += floatMems[5] * timePassed;
    /*
    Serial.println(angles[0]);
    Serial.println(angles[1]);
    Serial.println(angles[2]);
    */
}

void SafetyZeroing()
{
  //biztonsági okokból:
  for (byte i = 0; i<4; i++)
    {
      motoSpeed[i] = 0;
      motoThrust[i] = 0; 
      motoX[i] = 0;
      motoY[i] = 0;
    }
}

//****************************************************************************************
//Kommunikáció másik i2c-vel

//Semmi
//