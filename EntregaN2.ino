#include <Arduino.h>
#define CHAO 50
#define DISTANCIA 30
#define delay1 500            //delay
#define delay2 1000         //delay
#define delay3 1500       //delay
//#define distancia 30   //distancia
//#define chao 50       //chao



class Robot {
public:
  int motorEsquerdoTras;
  int motorEsquerdoFrente;
  int motorDireitoFrente;
  int motorDireitoTras;
  int motorDireitoDefine;
  int motorEsquerdoDefine;
  int sensorFrente;
  int sensorDireito;
  int sensorEsquerdo;
  int chao;
  int distancia;
  bool firstTime;

public:
  Robot(int pinMotorEsquerdoTras, int pinMotorEsquerdoFrente, int pinMotorDireitoFrente, int pinMotorDireitoTras,
        int pinMotorDireitoDefine, int pinMotorEsquerdoDefine, int pinSensorFrente, int pinSensorDireito, int pinSensorEsquerdo,
        int chaoThreshold, int distanciaThreshold)
    : motorEsquerdoTras(pinMotorEsquerdoTras), motorEsquerdoFrente(pinMotorEsquerdoFrente),
      motorDireitoFrente(pinMotorDireitoFrente), motorDireitoTras(pinMotorDireitoTras),
      motorDireitoDefine(pinMotorDireitoDefine), motorEsquerdoDefine(pinMotorEsquerdoDefine),
      sensorFrente(pinSensorFrente), sensorDireito(pinSensorDireito), sensorEsquerdo(pinSensorEsquerdo),
      chao(chaoThreshold), distancia(distanciaThreshold), firstTime(true) {
    pinMode(motorDireitoDefine, OUTPUT);
    pinMode(motorEsquerdoDefine, OUTPUT);
    pinMode(sensorFrente, INPUT);
    pinMode(sensorDireito, INPUT);
    pinMode(sensorEsquerdo, INPUT);
    motorSetup();
  }

  void motorSetup() {
    digitalWrite(motorDireitoDefine, HIGH);
    digitalWrite(motorEsquerdoDefine, HIGH);
  }

  void motorOn(int motorPin) {
    digitalWrite(motorPin, HIGH);
  }

  void motorOff(int motorPin) {
    digitalWrite(motorPin, LOW);
  }

  int readSensor(int sensorPin) {
    return analogRead(sensorPin);
  }

  bool linhaBranca(int leituraSensor) {
    return leituraSensor >= chao;
  }

  bool inimigoAFrente(int leituraSensor) {
    return leituraSensor <= distancia;
  }

  void girarOn(int motorFrente, int motorTras) {
    motorOn(motorFrente);
    motorOn(motorTras);
  }

  void girarOff(int motorFrente, int motorTras) {
    motorOff(motorFrente);
    motorOff(motorTras);
  }

  void girarPraProcurarOponente(bool inimigoAFrente, int motorFrente, int motorTras) {
    if (inimigoAFrente) {
      girarOff(motorFrente, motorTras);
      motorOn(motorDireitoFrente);
      motorOn(motorEsquerdoFrente);
    } else {
      motorOff(motorDireitoFrente);
      motorOff(motorEsquerdoFrente);
      girarOn(motorFrente, motorTras);
    }
  }

  void mainFunction() {
    if (!linhaBranca(readSensor(sensorEsquerdo)) || !linhaBranca(readSensor(sensorDireito))) {
      motorOff(motorDireitoTras);
      motorOff(motorEsquerdoTras);

      if (!inimigoAFrente(readSensor(sensorFrente))) {
        motorOff(motorDireitoFrente);
        motorOff(motorEsquerdoFrente);
        
        analogWrite(motorDireitoFrente, delay1);
        analogWrite(motorEsquerdoTras, delay1);

        firstTime = false;
      } else {
        if (firstTime) {
          girarOff(motorDireitoFrente, motorEsquerdoTras);
          girarOn(motorDireitoTras, motorEsquerdoFrente);
          delay(500);
          girarOff(motorDireitoTras, motorEsquerdoFrente);
        }
        
        firstTime = true;
        
        motorOn(motorDireitoFrente);
        motorOn(motorEsquerdoFrente);
      }
    } else {
      motorOff(motorDireitoFrente);
      motorOff(motorEsquerdoFrente);
      girarOff(motorDireitoFrente, motorEsquerdoTras);
      motorOn(motorDireitoTras);
      motorOn(motorEsquerdoTras);
      //delay(delay3);
    }
  }
};


const int pinMotorEsquerdoTras = 8;
const int pinMotorEsquerdoFrente = 7;
const int pinMotorDireitoFrente = 5;
const int pinMotorDireitoTras = 4;
const int pinMotorDireitoDefine = 3;
const int pinMotorEsquerdoDefine = 6;
const int pinSensorFrente = A5;
const int pinSensorDireito = A0;
const int pinSensorEsquerdo = A2;


Robot robot(pinMotorEsquerdoTras, pinMotorEsquerdoFrente, pinMotorDireitoFrente, pinMotorDireitoTras,
            pinMotorDireitoDefine, pinMotorEsquerdoDefine, pinSensorFrente, pinSensorDireito, pinSensorEsquerdo,
            50, 30);

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(robot.readSensor(robot.sensorFrente));
  robot.mainFunction();
}