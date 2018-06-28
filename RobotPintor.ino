/* Robot móvil con un servo y dos motores paso a paso controlados desde el driver ULN2003A que permiten establecer una localización por odometría y escribir textos.
   Diseño de las piezas 3D de MakersBox https://www.thingiverse.com/MakersBox/about.
   Autor: Javier Vargas. El Hormiguero.
   https://creativecommons.org/licenses/by/4.0/
*/
//PINES
#define PinServo 4
#define PinBoton 3
#define IN1_DER 12
#define IN2_DER 11
#define IN3_DER 10
#define IN4_DER 9
#define IN1_IZQ  19
#define IN2_IZQ  18
#define IN3_IZQ  17
#define IN4_IZQ  16

//CONFIGURACION
#define Perimetro 20.42f //(cm) Perimetro de la rueda
#define RadioRobot 5.75f //(cm) Desde la rueda al centro del robot
#define PasosVuelta 4096 //Pasos por vuelta del motor
#define Tmin 400//(us) tiempo minimo entre pasos
#define Tmax 1000 //(us) Tiempo maximo entre pasos
#define PasosVel 400 //Pasos en los que pasa de Tmax a Tmin (aceleracion)
#define AngPenUP 110 //Angulo servo sin dibujar
#define AngPenDOWN 150 //Angulo servo dibujando
#define Escala 1 //Factor de escala de las letras

const int Paso [ 8 ][ 4 ] = //Control del driver para dar medios pasos (Half Step)
{ {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};
#define GradToRad 0.01745f
#define RadToGrad 57.29577f
#include <Servo.h>
Servo servo;

#include <math.h>
double x, y = 0; //Posicion actual
double a = 0; //Angulo actual
boolean s = 0; //Servo

void setup() {
  //Serial.begin(115200);
  pinMode(PinBoton, INPUT_PULLUP);
  pinMode(IN1_IZQ, OUTPUT);
  pinMode(IN2_IZQ, OUTPUT);
  pinMode(IN3_IZQ, OUTPUT);
  pinMode(IN4_IZQ, OUTPUT);
  pinMode(IN1_DER, OUTPUT);
  pinMode(IN2_DER, OUTPUT);
  pinMode(IN3_DER, OUTPUT);
  pinMode(IN4_DER, OUTPUT);

  servo.attach(PinServo);
  PenUP();
}

//////////////////////////
//////////LOOP////////////
//////////////////////////

void loop() {
  if (Boton()) {
    delay(1000);
    Escribir("texto");
    //Triangulos(1);
    //Cuadrados(40, 1);
  }

}

//////////////////////////
//////////////////////////
//////////////////////////

boolean Boton() {
  return !digitalRead(PinBoton);
}

void Avanzar(float d) {
  //Pasos necesarios
  unsigned long pasos = d * (float)PasosVuelta / Perimetro;
  //Ejecutar pasos
  for (unsigned long p = 0; p < pasos; p++) {
    StepIZQ(0);
    StepDER(1);
    //Velocidad variable conforme nos acermaos/alejamos del punto de destino/origen
    int K = constrain(min(pasos - p, p), 0, PasosVel);
    int t = map(K, 0, PasosVel, Tmax, Tmin);
    delayMicroseconds(t);
  }
}

void GirarDer(float ang) {
  //Pasos necesarios
  float d = GradToRad * ang * RadioRobot;
  unsigned long pasos = d * (float)PasosVuelta / Perimetro;
  //Ejecutar pasos
  for (unsigned long p = 0; p < pasos; p++) {
    StepIZQ(0);
    StepDER(0);
    //Velocidad variable conforme nos acermaos/alejamos del punto de destino/origen
    int K = constrain(min(pasos - p, p), 0, PasosVel);
    int t = map(K, 0, PasosVel, Tmax, Tmin);
    delayMicroseconds(t);
  }
}

void GirarIzq(float ang) {
  //Pasos necesarios
  float d = GradToRad * ang * RadioRobot;
  unsigned long pasos = d * (float)PasosVuelta / Perimetro;
  //Ejecutar pasos
  for (unsigned long p = 0; p < pasos; p++) {
    StepIZQ(1);
    StepDER(1);
    //Velocidad variable conforme nos acermaos/alejamos del punto de destino/origen
    int K = constrain(min(pasos - p, p), 0, PasosVel);
    int t = map(K, 0, PasosVel, Tmax, Tmin);
    delayMicroseconds(t);
  }
}

void StepIZQ(boolean dir)  {   //Avanza un paso
  static byte s = 0;
  digitalWrite(IN1_IZQ, Paso[s][0]);
  digitalWrite(IN2_IZQ, Paso[s][1]);
  digitalWrite(IN3_IZQ, Paso[s][2]);
  digitalWrite(IN4_IZQ, Paso[s][3]);
  if (dir) s++;
  else s--;
  s = (s + 8) % 8; //Si s > 7 --> s = 0, si s < 0, s = 7;
}

void StepDER(boolean dir)  {   //Avanza un paso
  static byte s = 0;
  digitalWrite(IN1_DER, Paso[s][0]);
  digitalWrite(IN2_DER, Paso[s][1]);
  digitalWrite(IN3_DER, Paso[s][2]);
  digitalWrite(IN4_DER, Paso[s][3]);
  if (dir) s++;
  else s--;
  s = (s + 8) % 8; //Si s > 7 --> s = 0, si s < 0, s = 7;
}

void PenUP() {
  s = 0;
  servo.write(AngPenUP);
  delay(200);
}

void PenDOWN() {
  s = 1;
  servo.write(AngPenDOWN);
  delay(200);
}

void Go(float xf, float yf, boolean p) {
  float xdif = xf - x;
  float ydif = yf - y;

  if (xdif != 0 || ydif != 0) {

    //Calculo del angulo y distancia a avanzar
    float tg = (float)ydif / xdif; //Tangente
    float ang;
    if (xdif != 0) ang = RadToGrad * atan(tg); //Arcotangente
    else if (ydif > 0) ang = 90;
    else if (ydif < 0) ang = -90;
    float d = sqrt(xdif * xdif + ydif * ydif); //Hipotenusa
    if (xdif < 0 && ydif < 0) { //Tercer cuadrante
      ang = -(180 - ang);
    }
    else if (xdif < 0 && ydif >= 0) { //Cuarto cuadrante
      ang = 180 + ang;
    }

    //Diferencia de angulo a girar
    float difang = ang - a;
    if (difang < -180) difang = 360 + ang - a;
    if (difang > 180) difang = -(360 + a - ang);

    //Pintar
    if (p && !s) PenDOWN(); //Bajar boli
    if (!p && s) PenUP(); //Subir boli
    //Movimiento
    if (difang > 0) GirarIzq(difang);
    else GirarDer(abs(difang));
    Avanzar(d * Escala);
    x = xf;
    y = yf;
    a = ang;
  }
}

void ResetPos() {
  x = 0;
  y = 0;

  //Diferencia de angulo a girar
  float difang = - a;
  if (difang < -180) difang = 360 - a;
  if (difang > 180) difang = -(360 + a);
  //Movimiento
  if (difang > 0) GirarIzq(difang);
  else GirarDer(abs(difang));

  a = 0;
}

void Escribir(String texto) {
  for (int i = 0; i < texto.length(); i++) {
    char letra = texto[i];
    Letra((String)letra);
  }
  Letra("fin");
}

///////////////FIGURAS/////////////////

void Cuadrados(int n, float e) {
  float dmax = 10 * e;
  float dmin = e;
  float paso = (float)(dmax - dmin) / n;
  PenDOWN();
  for (int i = 0; i < n; i++) {
    Avanzar(dmax - paso * i);
    GirarDer(92);
  }
  PenUP();
}

void Triangulos(float e) {
  float dmax = 10 * e;
  PenDOWN();
  for (int i = 0; i < 20 ; i++) {
    Avanzar(dmax);
    GirarDer(125);
  }
  PenUP();
}

///////////////LETRAS/////////////////

void Letra(String s) {
  if (s == "a") {
    Go(3, 6, 1);
    Go(6, 0, 1);
    Go(1.75, 3, 0);
    Go(4.25, 3, 1);
    Go(7, 0, 0);
  }
  else if (s == "b") {
    Go(0, 6, 1);
    Go(3, 4.5, 1);
    Go(0, 3, 1);
    Go(3, 1.5, 1);
    Go(0, 0, 1);
    Go(4, 0, 0);
  }
  else if (s == "c") {
    Go(4, 6, 0);
    Go(0, 6, 1);
    Go(0, 0, 1);
    Go(4, 0, 1);
    Go(5, 0, 0);
  }
  else if (s == "d") {
    Go(0, 6, 1);
    Go(4, 3, 1);
    Go(0, 0, 1);
    Go(5, 0, 0);
  }
  else if (s == "e") {
    Go(4, 0, 0);
    Go(0, 0, 1);
    Go(0, 6, 1);
    Go(4, 6, 1);
    Go(0, 3, 0);
    Go(2, 3, 1);
    Go(5, 0, 0);
  }
  else if (s == "f") {
    Go(0, 6, 1);
    Go(4, 6, 1);
    Go(0, 3, 0);
    Go(3, 3, 1);
    Go(5, 0, 0);
  }
  else if (s == "g") {
    Go(4, 6, 0);
    Go(0, 6, 1);
    Go(0, 0, 1);
    Go(4, 0, 1);
    Go(4, 3, 1);
    Go(2, 3, 1);
    Go(5, 0, 0);
  }
  else if (s == "h") {
    Go(0, 6, 1);
    Go(4, 6, 0);
    Go(4, 0, 1);
    Go(0, 3, 0);
    Go(4, 3, 1);
    Go(5, 0, 0);
  }
  else if (s == "i") {
    Go(0, 6, 1);
    Go(2, 0, 0);
  }
  else if (s == "j") {
    Go(0, 2, 0);
    Go(0, 0, 1);
    Go(4, 0, 1);
    Go(4, 6, 1);
    Go(5, 0, 0);
  }
  else if (s == "l") {
    Go(0, 6, 0);
    Go(0, 0, 1);
    Go(4, 0, 1);
    Go(5, 0, 0);
  }
  else if (s == "m") {
    Go(2, 6, 1);
    Go(4, 4, 1);
    Go(6, 6, 1);
    Go(8, 0, 1);
    Go(9, 0, 0);
  }
  else if (s == "n") {
    Go(0, 6, 1);
    Go(4, 0, 1);
    Go(4, 6, 1);
    Go(5, 0, 0);
  }
  else if (s == "o") {
    Go(4, 0, 1);
    Go(4, 6, 1);
    Go(0, 6, 1);
    Go(0, 0, 1);
    Go(5, 0, 0);
  }
  else if (s == "p") {
    Go(0, 6, 1);
    Go(4, 6, 1);
    Go(4, 3, 1);
    Go(0, 3, 1);
    Go(5, 0, 0);
  }

  else if (s == "q") {
    Go(4, 0, 1);
    Go(4, 6, 1);
    Go(0, 6, 1);
    Go(0, 0, 1);
    Go(2, 2, 0);
    Go(5, 1, 1);
    Go(6, 0, 0);
  }
  else if (s == "r") {
    Go(0, 6, 1);
    Go(4, 6, 1);
    Go(4, 3, 1);
    Go(0, 3, 1);
    Go(4, 0, 1);
    Go(5, 0, 0);
  }
  else if (s == "s") {
    Go(4, 0, 1);
    Go(4, 3, 1);
    Go(0, 3, 1);
    Go(0, 6, 1);
    Go(4, 6, 1);
    Go(5, 0, 0);
  }
  else if (s == "t") {
    Go(2, 0, 0);
    Go(2, 6, 1);
    Go(0, 6, 0);
    Go(4, 6, 1);
    Go(5, 0, 0);
  }
  else if (s == "u") {
    Go(4, 0, 1);
    Go(4, 6, 1);
    Go(0, 6, 0);
    Go(0, 0, 1);
    Go(5, 0, 0);
  }
  else if (s == "v") {
    Go(0, 6, 0);
    Go(2, 0, 1);
    Go(4, 6, 1);
    Go(5, 0, 0);
  }
  else if (s == "y") {
    Go(2, 0, 0);
    Go(2, 3, 1);
    Go(0, 6, 1);
    Go(4, 6, 0);
    Go(2, 3, 1);
    Go(5, 0, 0);
  }
  else if (s == " ") {
    Go(5, 0, 0);
  }
  else if (s == "fin") {
    Go(15, 0, 0);
  }
  ResetPos();
}

