/*
   Hiago Riba Guedes
*/
 #include <SimpleKalmanFilter.h>
SimpleKalmanFilter simpleKalmanFilter(2, 100, 0.01);

// --- Bibliotecas Auxiliares ---
#include <AFMotor.h> //Inclui biblioteca AF Motor
 

// --- Seleção dos Motores ---
AF_DCMotor motor1(3); //Seleção do Motor 1
AF_DCMotor motor2(4); //Seleção do Motor 1

const int clkPinA = 2;
const int dirPinA = 23;
const int clkPinB = 3;
const int dirPinB = 24;
// --- Variáveis Globais ---
unsigned char velocidade1 = 0x00; //Armazena a velocidade do motor1 (8 bits)
 #define power 22
 volatile int encoderACount = 0;
volatile int encoderBCount = 0;
  unsigned long timeold = 0;
int rpm_l,rpm_r;
 float nx,kx;
float ny,ky;
float ntheta,ktheta;

// --- Configurações Iniciais ---
void setup()
{
   digitalWrite(power,HIGH);
  attachInterrupt(2, encoderIntA, RISING);
  attachInterrupt(3, encoderIntB, RISING);/*
  pinMode(dirPinA, INPUT); 
  pinMode(dirPinB, INPUT); */
  timeold=0;
  nx=0;ny=0;ntheta=0;
 rpm_l=0;rpm_r=0;
 velocidade1 = 0xFF;           //Velocidade recebe o valor máximo

}
// --- Loop Infinito ---
void loop()
{

             
      motor1.setSpeed(velocidade1); //Seleciona velocidade atual
      motor1.run(FORWARD);          //Movimenta motor no sentido horário
      motor2.setSpeed(velocidade1); //Seleciona velocidade atual
      motor2.run(FORWARD);          //Movimenta motor no sentido horário
    /*  velocidade1 = 0x00;           //Velocidade recebe o valor mínimo
      motor1.setSpeed(velocidade1); //Seleciona velocidade atual
      motor1.run(RELEASE);          //Motor parado
      delay(2000);                  //Mantém por 2 segundos
      
      
      velocidade1 = 0xFF;           //Velocidade recebe o valor máximo
      motor1.setSpeed(velocidade1); //Seleciona velocidade atual
      motor1.run(BACKWARD);         //Movimenta motor no sentido anti-horário
      delay(2000);                  //Mantém por 2 segundos
      
      velocidade1 = 0x00;           //Velocidade recebe o valor mínimo
      motor1.setSpeed(velocidade1); //Seleciona velocidade atual
      motor1.run(RELEASE);          //Motor parado
      delay(2000);                  //Mantém por 2 segundos*/

       


} 

void encoderIntA() {
  if (digitalRead(dirPinA) == LOW)
    encoderACount++;
}

void encoderIntB() {
  if (digitalRead(dirPinB) == LOW)
    encoderBCount++;

}

void update_posicao(int wl,int wr){

int dr_hist;
int dl_hist;
float dr=2*3.1415* (wr);
float dl=2*3.1415* (wl);
  ntheta+=(dr-dl)/(2*0.125)*(3.1415/180);
  nx+=(dr+dl)/2 * cos(ntheta+((dr-dr_hist-(dl-dl_hist))/(2*0.125)));
  ny+=(dr+dl)/2 * sin(ntheta+((dr-dr_hist-dl-dl_hist)/(2*0.125)));
   dr_hist=dr;
   dl_hist=dl;

  if (ntheta > 2.0 * PI) ntheta -= 2.0 * PI;
  if (ntheta < 0.0) ntheta += 2.0 * PI;

Serial.print("\t");Serial.print("\t");Serial.print("\t");Serial.print("\t");Serial.print("\t");
;  Serial.print("points.add(");Serial.print(nx);
Serial.print("\t");Serial.print(",");
;Serial.print(ny);Serial.print(");");
Serial.print("\t");Serial.println(); //Serial.print("theta = ");Serial.println(ktheta);


}

 void updatePosicao_Kalman(int wl,int wr, double P,double Q,double R){

  float hist_theta=ktheta;
  float dr_hist;
  float dl_hist;
  float lastUpdate;
  float const th=1000;
  
float dr=2*3.1415* (wr);
float dl=2*3.1415* (wl);
  ktheta+=(dr-dl)/(2*0.125)*(3.1415/180);
  kx+=(dr+dl)/2 * cos(ktheta+((dr-dr_hist-(dl-dl_hist))/(2*0.125)));
  ky+=(dr+dl)/2 * sin(ktheta+((dr-dr_hist-dl-dl_hist)/(2*0.125)));

  float Delta_theta = ktheta-hist_theta;
  
  if (ktheta > 2.0 * PI) ktheta -= 2.0 * PI;
  if (ktheta < 0.0) ktheta += 2.0 * PI;

  //double dLeft = leftDegrees / _degreesPerMillimeter;
  //double dRight = rightDegrees / _degreesPerMillimeter;
  //double dCenter = (kx + ky) / 2.0;

double A22=ktheta+Delta_theta/2;
double A00= kx; double A02=(dr+dl)*0.5*cos(A22);
double A11=ky;  double A12=(dr+dl)*0.5*sin(A22);


   float  timeNow = millis ();
   float  dt = (timeNow -  lastUpdate)/th ;
     lastUpdate = millis ();

kx=A00*kx +A02*ktheta;
ky=A11*ky +A12*ktheta;
ktheta=A22*ktheta;

double P00,//P01,P02,
       P10,P11,P12,
       P20,P21,P22;

P00=P+P00;
P11=P+P11;
P22=P+P22;
  
P00+=P00*kx*P00+Q;
P11+=P11*ky*P11+Q;
P22+=P22*ktheta*P22+Q;

double K0,K1,K2;

K0=P00/(P00+R);
K1=P11/(P11+R);
K2=P22/(P22+R);


kx = K0*(((-wr*0.035)/dt)+kx);
ky= K1*(((-wl*0.035)/dt)+ky);
ktheta = K2*((-(wr-wl)/dt) +ktheta);

  P00 *= (1 - K0);
  P11 *= (1 - K1);
  P22 *= (1 - K2);

dr_hist=dr;
dl_hist=dl;
Serial.print("\t");Serial.print("\t");Serial.print("\t");Serial.print("\t");Serial.print("\t");
;  Serial.print("points.add(");Serial.print(kx);
Serial.print("\t");Serial.print(",");
;Serial.print(ky);Serial.print(");");
Serial.print("\t");Serial.println(); //Serial.print("theta = ");Serial.println(ktheta);
   }
  











