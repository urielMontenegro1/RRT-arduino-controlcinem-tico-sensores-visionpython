/*Algoritmo basado elaborado por urielMontenegro1 para cualquier asunto consultar al correo:urielmontenegro22@gmail.com
 Estudiante de  ingeniería robótica, en la universidad de Guadalajara(UDG)
 
 Para futuro se planean dar mejoras y continuaciones a proyecto completo en un robot movil, asi tambien como haciendo más eficiente
 su funcionamiento y compactamiento.
 ATENCIÓN: EL USO DEL CODIGO ES UNICAMENTE PARA QUE EL USUARIO APROVECHE ESTE METODO DE UNA MANERA RESPONSABLE, EL AUTOR NO SE
 RESPONSABILIZA  DEL DAÑO QUE PUEDA  GENERAR EL USUARIO EN TURNO, ES TOTALMENTE RESPONSABILIDAD DEL USUARIO. DIVIERTETE.  
 */

 
/*El uso de este codigo esta pensado para robots móviles para generar dentro de un arbol de grafos generados, mediante puntos 
 * aleatorios, encuentre un camino hasta sus posiciones deseadas, recuerda configurar  la selecciones de opciones,asi como 
 * sus posiciones deseadas,sus limites superior e inferior, la distancia entre grafos y el tamaño de los vectores acorde al 
 * numero de iteraciones(N).
 */

//CONTAMOS QUE SUS PULSOS son de 22 O TICKS SON 748 POR CADA VUELTA de nuestos encoders


/////////////////////////////////////////////////////////LIBRERIAS
#include "math.h"
///////////////////////////////////////////////////////////////////////////////////
//configuracion de posiciones deseadas en x y y
float xd = 200.0+50.0; //cm deseada +50 para  darle una trayectoria m´´as acertada
float yd = 150.0*2.0; //cm   + 30 o*2
int opc; //opcion a elegir de trayetoria
int activacion=0;

int  opcion;  
//garancias
//float kv=0.5,kw=5.0;,float kv=0.5,kw=10.0;kv=0.5,kw=1.0;
float kv,kw;//ganancias



int i18 = 1;//iteraciones
//float error=30;,50,40
float error;
int cont4 = 0;//iteraciones

//int minimoVelocidad=10;//11
//int maximoVelocidad=50;
//configuraciones para tener un estimado entre dos velocidades
int minimoVelocidad;//10,
int maximoVelocidad;//60,
int velocidadAtras;//40
int boton =22;// se configura boton para reiniciar dentro de las trayectorias los datos de pasos, posiciones, etc.

///estas configuraciones son para los sensores de proximidad
float f_y, f_yn,f_y2, f_yn2,f_y3, f_yn3;
int randoms;

//////////////////////////////////////////////////////////////////SENSORES
//sensor 1
//const int Trigger1 = 18;   //Pin digital 2 para el Trigger del sensor
//const int Echo1 = 35;   //Pin digital 3 para el Echo del sensor
//
////Sensor 2
//const int Trigger2 = 19 ;  //Pin digital 2 para el Trigger del sensor
//const int Echo2 = 34;   //Pin digital 3 para el Echo del sensor
//
////Sensor 3
//const int Trigger3 = 32;  //Pin digital 2 para el Trigger del sensor
//const int Echo3 = 39;   //Pin digital 3 para el Echo del sensor

/// delimitadores de los sensores de proximidad
///mide hasta donde no debe de par, mide hasta donde  va a empezar a estimar un objeto, y cuanto tiempo mide ese objeto
int deteccionLimite = 20, minimoDeteccion =60,cont2=0;

float s1, s2, s3, xs1, xs2, xs3, ys1, ys2, ys3, radio = 20;
/////////////////////////////////////////////////////////////////////parametros de carro
float radioRueda = 3.3; // en cm
float baseRuedas = 22; //en cm
// Motor A
// Motor derecha //configruarcion ESP32 ///
int motor1Pin1 = 5;
int motor1Pin2 = 26;
int enable1Pin = 14;

//Motor izquierda
int motor2Pin1 = 25;
int motor2Pin2 = 33;
int enable2Pin = 15;

// Setting PWM properties
const int freq = 30000;

const int pwmChannel = 0;
const int pwmChannel2 = 1;
const int resolution = 8;

int dutyCycle = 50;//30 esta bien


   


////////////////////////////////////////////////////////////////////////////////////// ENCODERS


const byte Encoder_C1d = 21; // Cable amarillo 
const byte Encoder_C2d = 4; // Cable verde 
const byte Encoder_C1i = 27; // Cable verde 
const byte Encoder_C2i = 13; // Cable amarillo 

byte Encoder_C1Lastd, Encoder_C1Lasti;
int pasoD, pasoI;
boolean direcciond, direccioni;
float dc, dcw, dr, di;
int stby = 23;





//posiciones inicial

float x_0 = 0;
float y_0 = 0;

//definir distancia D
float D = 20;

//////////////////////////////////////////////////////////////////////////////////
//limite inferior
float xl = -400;
float yl = -400;
//limite superior
float xu = 400;
float yu = 400;

//numero de iteraciones
int N = 100;

//ALMACENA LAS COORDENADAS DEL GRAFO
float xr;
float yr;
float da;

float d;
int j = 1, num = 0;
float nor;
float d_min;
int i_min;
int  indice2[101], padres[101];
float mapax1[101], mapay1[101], mapax2[101], mapay2[101];
int target2[101];
float Mx[101];
float My[101];
float xi[101];
float yi[101];
float vx[101];
float vy[101];
float v2;

float xc[101];
float yc[101];
int i7 = 0, i8;
int i10 = 0, i11 = 0;
boolean resultado;
float vu;
float vv;

int hijos[101], cont = 0;
//////////////////////////////////////////////////////////////////////////////////////////
float alpha, betha, orientacion[101];

float psx[101], psy[101], itx[101], ity[101];
int NT = 10;
float mapaxf[100];
float mapayf[100];

/////////////////////////////////////////////////////////////////////////////////////CONTROL

float posx = 0, posy = 0, posw = 0,posd=0;
float xdf=0, ydf=0, ev=0, ew=0, v=0, w=0, wr=0, wl=0;



void setup() {
     opc=1;
  
     if(opc==1){
       kv=1;
       kw=10;
       
       error=65;
       
       minimoVelocidad=10;//10,
       maximoVelocidad=65;//60,
       velocidadAtras=40;//40
      }
     else if(opc==2){
       kv=0.5;
       kw=1.1;
       error=45;
       minimoVelocidad=10;//10,
       maximoVelocidad=65;//60,
       velocidadAtras=40;//40
      } 
      
    
   pinMode(boton,INPUT);
 
  // sets the pins as outputs:
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  pinMode(stby, OUTPUT);
  
  // configure LED PWM functionalitites
    ledcSetup(pwmChannel, freq, resolution);
    ledcSetup(pwmChannel2, freq, resolution); 
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(enable1Pin, pwmChannel);
//  
    ledcAttachPin(enable2Pin, pwmChannel2);

  //////////////////////////////////////////////////////////////////////////sensor
  ////sensorres
//  pinMode(Trigger1, OUTPUT); //pin como salida
//  pinMode(Echo1, INPUT);  //pin como entrada
//  digitalWrite(Trigger1, LOW);//Inicializamos el pin con 0
//
//  pinMode(Trigger2, OUTPUT); //pin como salida
//  pinMode(Echo2, INPUT);  //pin como entrada
//  digitalWrite(Trigger2, LOW);//Inicializamos el pin con 0
//
//  pinMode(Trigger3, OUTPUT); //pin como salida
//  pinMode(Echo3, INPUT);  //pin como entrada
//  digitalWrite(Trigger3, LOW);//Inicializamos el pin con 0

  ///////////////////////////////////////////////////////////////////

  digitalWrite(stby, 1);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.begin(115200);


}



void loop() {

//delay(3000);
  while (j < (N + 1)) {

    float xn[j];
    float yn[j];

    vu = random(0, 101);
    vu = vu / 100;
    vv = random(0, 101);
    vv = vv / 100;
    //punto aleatorio x y y
    xr = xl + (xu - xl) * (vu); //Xr
    yr = yl + (yu - yl) * (vv); //Yr

    //           Serial.print("PUNTOS XR Y YR");
    //            Serial.println(" ");
    //          Serial.print(xr);
    //           Serial.print(",");
    //           Serial.print(yr);
    //           Serial.print(",");
    //           Serial.print(vu);
    //           Serial.print(",");
    //           Serial.print(vv);
    //            Serial.println(" ");


    d_min = 1000; //distancia que en el grafo este más cercano
    i_min = 0; //indice

    //                 Serial.print("d_min inicial y i_min inicial");
    //                  Serial.println(" ");
    //          Serial.print(d_min);
    //           Serial.print(",");
    //           Serial.print(i_min);
    //            Serial.println(" ");
    //
    //              delay(2000);
    /////////////////////ticks/748 es el numero de pasos cada 748 trics es un paso


    for (int i = 0; i < j; i++) {

      //POSICIONES ACTUALES
      Mx[0] = x_0;
      My[0] = y_0;
      xi[i] = Mx[i]; //Xi
      yi[i] = My[i]; //Yi


      //           Serial.print("posiciones xi y yi");
      //            Serial.println(" ");
      //          Serial.print(xi[i]); Serial.print(",");  Serial.print(yi[i]);
      //       Serial.println(" ");
      da = pow((xr - xi[i]), 2) + pow((yr - yi[i]), 2);
      d = sqrt(da);

      //                 Serial.print("distancia respecto a las posiciones xr y yr");
      //            Serial.println(" ");
      //          Serial.print(d);
      //       Serial.println(" ");
      //
      //       delay(2000);

      if (d < d_min) {
        d_min = d;
        i_min = i;
        indice2[j] = i;

      }



      //       Serial.print("distancia guardada, indice,idice2");
      //       Serial.println(" ");
      //       Serial.print(d_min);
      //       Serial.print(", ");
      //       Serial.print(i_min);
      //       Serial.print(", ");
      //       Serial.print(i_min_a);
      //       Serial.println(" ");
      //       delay(2000);
    }




    //se agrega punto más cercano
    xc[j] = Mx[i_min]; //Xc
    yc[j] = My[i_min]; //Yc

    //       Serial.print("punto cercano xc y yc");   Serial.println(" ");
    //       Serial.print(xc[j]);Serial.print(",");Serial.println(yc[j]);
    //       Serial.println(" ");
    //       delay(2000);

    vx[j] = xr - xc[j];
    vy[j] = yr - yc[j];
    v2 = pow((xr - xc[j]), 2) + pow((yr - yc[j]), 2);
    v2 = sqrt(v2);
    //        if(v2==0){
    //        vx[]=0;
    //        vy[i]=0;
    //          }
    //          else{
    vx[j] = vx[j] / v2;
    vy[j] = vy[j] / v2;
    //}

    //       Serial.print("Vector Vx,vy");   Serial.println(" ");
    //       Serial.print(vx[j]);Serial.print(",");Serial.println(vy[j]);
    //       Serial.println(" ");
    //       delay(2000);
    //
    xn[j] = xc[j] + D * (vx[j]); //creamos la nueva posicion Xn
    yn[j] = yc[j] + D * (vy[j]); //creamos la nueva posicion Yn

    //       Serial.print("posiciones nuevas xn y yn");   Serial.println(" ");
    //       Serial.print(xn[j]);Serial.print(",");Serial.println(yn[j]);
    //       Serial.println(" ");
    //       delay(2000);


    Mx[j] = xn[j];
    My[j] = yn[j];

    //       Serial.print("posiciones nuevas grafos MX y My");   Serial.println(" ");
    //       Serial.print(Mx[j]);Serial.print(",");Serial.println(My[j]);
    //       Serial.println(" ");
    //       delay(2000);



    //       Serial.print("Source y target");   Serial.println(" ");
    //       Serial.print(S[j]);Serial.print(",");Serial.println(T[j]);
    //       Serial.println(" ");
    //       delay(2000);



    nor = pow((xd - xn[j]), 2) + pow((yd - yn[j]), 2);
    nor = sqrt(nor);
    //        ////////////////poner una condicion  que se acerrque sino reinicciar
    //       Serial.println("distancia de posicion xn y yn respecto a  xd,yd");
    //       Serial.print(nor);
    //       Serial.println(" ");
    //       delay(2000);

//    Serial.println(" ");
//
//    Serial.print("ciclo");
//    Serial.print("[");
//    Serial.print(j);
//    Serial.print("]");
//
//
//    Serial.println("");
//
//
//
//
//
//
//    Serial.println(" ");

    if (nor < D) { //si esta lo suficientemente cercano rompo el ciclo

      //y agregamos al grafos y listas de padres e hijos
      Mx[j + 1] = xd;
      My[j + 1] = yd;

//      Serial.print("Break"); Serial.println(" ");
//
//      Serial.print("[posición x:]");
//
//      for (int j1 = 0; j1 < j + 1; j1++) {
//        Serial.print("[");
//        Serial.print(j1);
//        Serial.print("]");
//        Serial.print("[");
//        Serial.print(Mx[j1]);
//        Serial.print("]");
//
//      }
//      Serial.print("[");
//      Serial.print(j + 1);
//      Serial.print("]");
//      Serial.print("[");
//      Serial.print( Mx[j + 1]);
//      Serial.print("]");
//
//      Serial.println(" ");
//
//      Serial.print("[posición y:]");

     // for (int i2 = 0; i2 < j + 1; i2++) {
       // Serial.print("[");
       // Serial.print(i2);
      //  Serial.print("]");
       // Serial.print("[");
      // Serial.print(My[i2]);
       // Serial.print("]");
      //}

      //Serial.print("[");
      //Serial.print(j + 1);
     // Serial.print("]");
     // Serial.print("[");
     // Serial.print( My[j + 1]);
     // Serial.print("]");
     // Serial.println(" ");

     // Serial.println(" ");

      //Serial.print("sources");

    //  for (int i4 = 0; i4 < j + 1; i4++) {
      //  Serial.print("[");
      //  Serial.print(indice2[i4]);
       // Serial.print("]");

     // }
     // Serial.println(" ");

     // Serial.print("targets");
      for (int i6 = 0; i6 < j + 1; i6++) {
        target2[j] = i6;
       // Serial.print("[");
       // Serial.print(target2[j]);
       // Serial.print("]");
      }

      //Serial.println(" ");


      /////////////////////////////////comparar con boleeanos

      i11 = j;
      for (i10 = j; i10 >= 0; i10--) {
        target2[j] = i10;

        int test1 = indice2[i11];
        int test2 = target2[j];
        //
        //Serial.print("resultado");
        if ( test1 == test2) {
          resultado = true;
          //     Serial.println(" ");

          //            Serial.print("[");
          //       Serial.print(resultado);
          //     Serial.print("]");
          cont = cont + 1;
          hijos[cont + 1] = target2[j];
          padres[cont] = indice2[i11];
          mapax1[cont] = Mx[i11];
          mapay1[cont] = My[i11];
          mapax2[cont + 1] = Mx[i11];
          //mapay2[cont + 1] = Mx[i11];
          mapay2[cont + 1] = My[i11];

          i8 = i11 - test2;
          i11 = i11 - i8;
        }


        //               Serial.print("padres e hijos");
        //            Serial.print("[");
        //       Serial.print(indice2[i11]);
        //     Serial.print("]");
        //          Serial.print(",");
        //                 Serial.print("[");
        //       Serial.print(target2[j]);
        //     Serial.print("]");




      }

      //Serial.println(" ");
      //Serial.print("padres");
     // for (int i12 = cont; i12 > 0; i12--) {

       // Serial.print("[");
       // Serial.print(padres[i12]);
       // Serial.print("]");

      //}

     // Serial.println(" ");
     // Serial.print("hijos");
      for (int i13 = cont; i13 > 0; i13--) {
        hijos[1] = j;
       // Serial.print("[");
       // Serial.print(hijos[i13]);
       // Serial.print("]");

      }
     // Serial.println(" ");
     // Serial.println(" ");
     // Serial.print("padres distancias");
//      for (int i14 = cont; i14 > 0; i14--) {

       // Serial.print("[");
       // Serial.print(mapax1[i14]);
       // Serial.print("]");
       // Serial.print("[");
       // Serial.print(mapay1[i14]);
        //Serial.print("]");
        //Serial.print(",");
//      }

      //Serial.println(" ");
      //Serial.print("hijos distanacias");
//      for (int i15 = cont; i15 > 0; i15--) {
//        mapax2[1] = xd;
//        mapay2[1] = yd;
        // Serial.print("[");
       // Serial.print(mapax2[i15]);
        //Serial.print("]");
       // Serial.print("[");
       // Serial.print(mapay2[i15]);
        //Serial.print("]");
       // Serial.print(",");
//      }
      //Serial.println(" ");

      //Serial.println(" ");

      mapaxf[cont];
      mapayf[cont];
      for (int i19 = 0; i19 <= cont; i19++) {

        mapaxf[i19] = mapax1[cont - i19];
        mapayf[i19] = mapay1[cont - i19];
      }
      //Serial.println(" ");



      //Serial.print("mapeado x,y");
      for (int i20 = 0; i20 <= cont; i20++) {
        mapaxf[cont] = xd;
        mapayf[cont] = yd;

          
        //Serial.print("[");
        Serial.print("posicion x:"+String(mapaxf[i20])+"\n");
        //Serial.print("]");
        //Serial.print("[");
        Serial.print("posicion y:" +String(mapayf[i20])+"\n");
        //Serial.print("]");
        //Serial.print(",");
      }
      //Serial.println(" ");

      /////////////////////////////////////////SUAVIZADO DE TRAYECTORIA

      for (int i16 = 0; i16 <= cont+1; i16++) {
        psx[i16] = mapaxf[i16];
        psy[i16] = mapayf[i16];
        alpha = 0.000099;
        betha = 0.099;
//        alpha = 0.0099;
//        betha = 0.099;
        //      alpha=0.03;
        //   betha=0.05;
        psx[i16] = psx[i16] + alpha * (mapax1[i16] - psx[i16]) + betha * (psx[i16 + 1] + psx[i16 - 1] - 2 * psx[i16]);
        psy[i16] = psy[i16] + alpha * (mapay1[i16] - psy[i16]) + betha * (psy[i16 + 1] + psy[i16 - 1] - 2 * psy[i16]);
       // Serial.println("mapa ");
        //Serial.print("[");
        //Serial.print(mapaxf[i16]);
        //Serial.print("]");  Serial.print("[");
        //Serial.print(mapayf[i16]);
        //Serial.print("]");
        //Serial.print(",numero:");
        //Serial.print(i16);
        //Serial.println(" ");
        //Serial.println("suavizado ");
        psx[cont+1]=xd;
        psy[cont+1]=yd;
  
        Serial.print("suavizado x:"+String(psx[i16])+"\n");
        //Serial.print(",");
        Serial.print("suavizado y:"+String(psy[i16])+"\n");




      }
      //Serial.println(" ");

      /////////////////////////////////ITERPOLACIÓN O AGREGAR PUNTOS

      //Serial.print("iterpolacion");
      float alphai;
      for (int i17 = 0; i17 <= cont; i17++) {
        cont4 = i17 * 3;
        for (int i21 = 1; i21 < 4; i21++) {
          alphai = i21 / 4.0;
         

          itx[cont4 + i21] = psx[i17 - 1] + alphai * (psx[i17] - psx[i17 - 1]);
          ity[cont4 + i21] = psy[i17 - 1] + alphai * (psy[i17] - psy[i17 - 1]);
          //                       Serial.println(" ");       Serial.print("alpha[");
          //       Serial.print(alphai);
          //     Serial.print("]");

          //Serial.println(" ");
          Serial.print("iterpolacion x:"+String(itx[cont4 + i21])+"\n");
          //Serial.print(",");
          Serial.print("iterpolacion y:"+String(ity[cont4 + i21])+"\n");
        }

      }
      //Serial.println(" ");

      //Serial.println(" ");

      //               Serial.print("Source y target");   Serial.println(" ");
      //       Serial.print(S[j]);Serial.print(",");Serial.println(T[j]);
      //       Serial.println(" ");
      //       delay(2000);

      j = j + N + 2;

      resultado = true;
      Serial.print("terminado RRT:");
      break;

    }


    //si la conexion no está dentro del rango no se pone falsa así que
    //prosigue a registrar


    j = j + 1; //es incremento de iteración

  }
  ////////////////////////////////////////////////////////////////////////////////////////

  //delay(3000);
   
  if (resultado == false ) {
    j = 0;
    cont = -1;
    cont4 = -1;

  }
  
if(opc==1){
  //opc=0;
  opcion=0;
  }
  
 
if (opc == 4) {
   opcion = cont4;
  } 
else {
    opcion = cont;
  }

  while (i18 <= opcion + 1) {
    
     if(digitalRead(boton)==0){     
       posx=0;
       posy=0;
       posw=0;
       ev=0;
       ew=0;
       v=0;
       w=0;
       wr=0;
       wl=0;
       pasoD=0;
       pasoI=0;
     }
    /////////////////////////////////////////////
//    long t1; //timepo que demora en llegar el eco
//    long d1; //distancia en centimetros
//    long s_1, s_2;
//    long t2; //timepo que demora en llegar el eco
//    long d2; //distancia en centimetros
//    long t3; //timepo que demora en llegar el eco
//    long d3; //distancia en centimetros
//    long vd, vi; //velocidades de las ruedas.

    



    //524
    //Serial.print("recorrido D :");
    //Serial.println(dr);
    //Serial.print("recorrido I :");
    //Serial.println(di);
    //Serial.print("recorrido dc :");
    //Serial.println(dc);

    ////////////////////////////////////////////////////////////////////////////////////////
//   //noInterrupts();
////    ////SENSOR 1
//    digitalWrite(Trigger1, HIGH);
//    delayMicroseconds(10);          //Enviamos un pulso de 10us
//    digitalWrite(Trigger1, LOW);
//
//    t1 = pulseIn(Echo1, HIGH); //obtenemos el ancho del pulso
//    d1 = t1 / 59;           //escalamos el tiempo a una distancia en cm
//
//    //Serial.print("Distancia sensor uno: ");
//    //Serial.print(d1);      //Enviamos serialmente el valor de la distancia
//    //Serial.print("cm");
//    //Serial.println();
////    //Hacemos una pausa de 100ms
////
////    ///SENSOR 2
//    digitalWrite(Trigger2, HIGH);
//    delayMicroseconds(10);          //Enviamos un pulso de 10us
//    digitalWrite(Trigger2, LOW);
//
//    t2 = pulseIn(Echo2, HIGH); //obtenemos el ancho del pulso
//    d2 = t2 / 59;           //escalamos el tiempo a una distancia en cm
//
//    //Serial.print("distancia sensor 2:" +String(d2)+"\n");
//    //Serial.print(d2);      //Enviamos serialmente el valor de la distancia
//    //Serial.print("cm");
//    //Serial.println();
//
//
//
////    ///SENSOR 3
//    digitalWrite(Trigger3, HIGH);
//    delayMicroseconds(10);          //Enviamos un pulso de 10us
//    digitalWrite(Trigger3, LOW);
//
//    t3 = pulseIn(Echo3, HIGH); //obtenemos el ancho del pulso
//    d3 = t3 / 59;           //escalamos el tiempo a una distancia en cm
//
//    //Serial.print("distancia sensor 3:" +String(d3)+"\n");
//    //Serial.print(d3);      //Enviamos serialmente el valor de la distancia
//    //Serial.println("cm");
//    //Serial.println();
//   delay(100);          //Hacemos una pausa de 100ms



    ///////////////////////////////////////////////////////////////////
    ////////////////CONTROL
     if(opc==1){
        //SOLO ES UN  PUNTO DADO EN X Y Y
        xdf = xd;
        ydf = yd;
        
      }
        
      if(opc==2){
        /// PUNTOS DE TRAYECTORIA DE RRT
        mapaxf[cont + 1] = xd;
        mapayf[cont + 1] = yd;
        xdf = mapaxf[i18];
        ydf = mapayf[i18];
        
      }
      
      if(opc==3){
        //PUNTOS DE TRAYECTORIA RRT SUAVIZADO
        psx[cont + 1] = xd;
        psy[cont + 1] = yd;
        xdf = psx[i18];
        ydf = psy[i18];
        
      }
      
       
      if(opc==4){
        //PUNTOS DE TRAYECTORIA RRT SUAVIZADO E ITERPOLADO
        itx[cont4 + 1] = xd;
        ity[cont4 + 1] = yd;
        xdf = itx[i18];
        ydf = ity[i18];
        
      }

     //if(xdf<0){
      //xdf=abs(xdf);
      //}

     //if(ydf<0){
      //ydf=abs(ydf);
      //}
    attachInterrupt(digitalPinToInterrupt(21), calculapulsoD, CHANGE);
    attachInterrupt(digitalPinToInterrupt(27), calculapulsoI, CHANGE);

    //Serial.print("Ángulo: "); //ANGULO DE RUEDA
    //Serial.println(paso * 0.481283422459893);

    /*Para por cada vuelta del rotor antes de la caja de engranes, se producen 22 pulsos (o pasos) del encoder.
      El motor tiene una relación de reducción en su caja de engranes de 1:34. Por tanto, se tienen 748 ticks o pulsos
      del encoder por cada revolución del rotor después de la caja de engranes. Por lo que 748/360 = 0.48128...*/

    

    dr = 2.0 * M_PI * radioRueda * (pasoD/748.0);
    di = 2.0 * M_PI * radioRueda * (pasoI/748.0);
    dc = (dr + di) / 2.0;

    posx = dc * cos(posw);
    posy = dc * sin(posw);
    posw = (dr - di) / baseRuedas;
    
      
    ev=sqrt((xdf-posx)*(xdf-posx)+(ydf-posy)*(ydf-posy));//error lineal
    ew=atan2((ydf-posy),(ydf-posx))-posw;//error angular
    ew = atan2(sin(ew), cos(ew));
    

    //velocidades 
    v = kv * ev;//velocidad lineal
    w = kw * ew;//velocidad angular

    wr = (2 * v + baseRuedas * w) / (2 * radioRueda);//velocidad de rueda derecha
    wl = (2 * v - baseRuedas * w) / (2 * radioRueda);//velocidad de rueda izquierda

    wr=abs(wr);
    wl=abs(wl);
    
    if(wr>=200){
      wr=200;
    }
    if(wl>=200) {
      wl=200;        
    }
    if(wr>=minimoVelocidad && wr<=maximoVelocidad){
      wr=maximoVelocidad;
    } 
    if(wr<=minimoVelocidad){
        delay(velocidadAtras);//40
       //Serial.println("atras derecha:");
        ledcWrite(pwmChannel, 70);
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
        //delay(velocidadAtras);
      }
    if(wl>=minimoVelocidad && wl<=maximoVelocidad){
      wl=maximoVelocidad;
    }
    if(wl<=minimoVelocidad){
       delay(velocidadAtras);
      //Serial.println("atras izquierda:");
         ledcWrite(pwmChannel2, 70);
         digitalWrite(motor2Pin1, LOW);
         digitalWrite(motor2Pin2, HIGH);
         //delay(velocidadAtras);
      }

     if(ev<=error){
         
       i18 = i18 + 1;
       
    Serial.print("pos x:" +String(posx)+"\n"); 
    Serial.print("pos y:" +String(posy)+"\n");
       //Serial.println("@"+String(posx)+","+String(posy)+"@");
       //Serial.println("! PERSONA ENCONTRADA"+"en"+String(posx)+","+String(posy)+"!")
       Serial.println("terminado:");
       
       wr=0;
       wl=0;
       stby = 0;
       
      } 

//      if( posx<=xdf+10 && ydf<=posy+10 && xdf>=posx-10 && ydf>=posy-10){
//         
//       i18 = i18 + 1;
//       
//       Serial.println("la posicion actual en X es:" + String(posx));
//       Serial.println("la posicion actual en Y es:" + String(posy));
//       //Serial.println("@"+String(posx)+","+String(posy)+"@");
//       //Serial.println("! PERSONA ENCONTRADA"+"en"+String(posx)+","+String(posy)+"!")
//       //Serial.println("terminado:");
//       
//       wr=0;
//       wl=0;
//       stby = 0;
//       
//      } 
      
          stby = 1;
    ledcWrite(pwmChannel, wr);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel2, wl);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);

       
    Serial.print("pos x:" +String(posx)+"\n"); 
    Serial.print("pos y:" +String(posy)+"\n");
    Serial.print("pos w:" +String(posw)+"\n");
    Serial.println("error:" +String(ev));
    Serial.println("velocidades:" +String(wr)+","+String(wl));
    Serial.println("a llegar:" +String(xdf)+","+String(ydf));
    

   //////////////////////////////DETECCION DE CIRCULOS, POSICIONES DE LOS CIRCULOS Y SUS ACOTAS MENORES,se checa si las distancias
   //estan dentro de esos rangos x y y para sino pasar al siguiente, pone objetos vituales a  10cm de radio
    //cálculo de las distancias y posiciones de los sensores
    //depende las posiciones de los sensortes es qeu van a detectar posiciones para checar
    //si el obstáculo está en mi caso estan  directo en poswy +- 55grados
//    if (d1 <= minimoDeteccion ) {
//      //sensor 1
//      //calculamos la distancia de la posicion  en que esta el robot
//      posd=sqrt((posx*posx)+(posy*posy));
//      //sumamos la posicion angular más donde se encuentra  el sensor en este caso 55°
//      s1 = posw + 55;
//      s1=(s1*M_PI)/180;//convertimos a radianes
//      //Serial.println("S1:");
//      //Serial.println((s1*180)/M_PI);
//      //calculamos sus posiciones en x y y del objeto
//      xs1 = cos(s1)*(d1 + posd + 20);
//      Serial.print("xs1:"+String(xs1)+"\n");
//      //Serial.println(xs1);
//      ys1 = tan(s1)*xs1;
//      Serial.print("ys1:"+String(ys1)+"\n");
//      //Serial.println(ys1);
//    }
//
//    if (d2 <= minimoDeteccion) {
//      //sensor 2
//      posd=sqrt((posx*posx)+(posy*posy));
//      s2 = posw - 55;
//      s2=(s2*M_PI)/180;
//      xs2 = cos(s2) * (d2 + posd + 20);
//      ys2 = tan(s2) * xs2;
//
//      Serial.print("xs2:"+String(xs2)+"\n");
//      Serial.print("ys2:"+String(ys2)+"\n");
//    }
//
//    if (d3 <= minimoDeteccion) {
//      //sensor 3
//      posd=sqrt((posx*posx)+(posy*posy));
//      s3 = posw;
//      s3=(s3*M_PI)/180;
//      xs3 = cos(s3) * (d3 + posd + 20);
//      ys3 = tan(s3) * xs3;
//
//      Serial.print("xs3:"+String(xs3)+"\n");
//      Serial.print("ys3:"+String(ys3)+"\n");
//    }
//   
//          if(d1<=minimoDeteccion){
//              //Serial.println("objeto detectado en x,y por sensor 1");
//              //Serial.print(xs1);Serial.print(",");Serial.println(ys1);
//    
//              //Serial.println("a distancia");
//              //Serial.println(d1);
////             GUARDA LOS PUNTOS EN UNA CIRCUFERENCIA DE DONDE LO DETECTÓ
//                     for(float x_1=xs1-radio;x_1<=xs1+radio;x_1++){
//            
//               f_y=sqrt((radio*radio)-((x_1-xs1)*(x_1-xs1)))+ys1;//menor a
//               f_yn=-sqrt((radio*radio)-((x_1-xs1)*(x_1-xs1)))+ys1; //mayor a
//               //Serial.println("puntos donde es circulo,x y y");
//               //Serial.print( f_y);Serial.print(",");;Serial.print(f_yn);Serial.print(",");;Serial.println(x_1);
//
//               //se compara si hay algun punto a llegar dentro del circulo para crear uuna nueva pposicion
//               if(xdf<=x_1+radio && xdf>=x_1-radio && ydf<=f_y && ydf>=f_yn){
//                  //Serial.println("recálculando...");
//   
//                  xdf=xs1;
//                  ydf=ys1-radio-20;
//                  Serial.print("nueva posicion x:"+String(xdf)+"\n");
//                  Serial.print("nueva posicion y:"+String(ydf)+"\n");
//                  //Serial.println("nuevas posiciones x y y");
//                  //Serial.print(xdf);Serial.print(",");Serial.println(ydf);
//              } 
//            }
//         }
////
//       if(d2<=minimoDeteccion){
//              //Serial.println("objeto detectado en x,y por sensor 2");
//              //Serial.print(xs2);Serial.print(",");Serial.println(ys2);
////    
//              //Serial.println("a distancia");
//              //Serial.println(d2);
//            // GUARDA LOS PUNTOS EN UNA CIRCUFERENCIA DE DONDE LO DETECTÓ
//                     for(int x_2=xs2-radio;x_2<=xs2+radio;x_2++){
//            
//               f_y2=sqrt((radio*radio)-((x_2-xs2)*(x_2-xs2)))+ys2;//menor a
//               f_yn2=-sqrt((radio*radio)-((x_2-xs2)*(x_2-xs2)))+ys2; //mayor a
//               //Serial.println("puntos donde es circulo,x y y");
//               //Serial.print( f_y2);Serial.print(",");;Serial.print(f_yn2);Serial.print(",");;Serial.println(x_2);
//
//               if(xdf<=x_2+radio && xdf>=x_2-radio && ydf<=f_y2 && ydf>=f_yn2){
//               //Serial.println("recálculando...");
//                 xdf=xs2;
//                 ydf=ys2+radio+20;
//                Serial.print("nueva posicion x:"+String(xdf)+"\n");
//                Serial.print("nueva posicion y:"+String(ydf)+"\n");
//                //Serial.print(xdf);Serial.print(",");Serial.println(ydf);
//                }    
//            }           
//         }
////
////
//                if(d3<=minimoDeteccion){
//              //Serial.println("objeto detectado en x,y por sensor 3");
//              //Serial.print(xs3);Serial.print(",");Serial.println(ys3);
//    
//              //Serial.println("a distancia");
//              //Serial.println(d3);
//            // GUARDA LOS PUNTOS EN UNA CIRCUFERENCIA DE DONDE LO DETECTÓ
//                     for(int x_3=xs3-radio;x_3<=xs3+radio;x_3++){
//            
//               f_y3=sqrt((radio*radio)-((x_3-xs3)*(x_3-xs3)))+ys3;//menor a
//               f_yn3=-sqrt((radio*radio)-((x_3-xs3)*(x_3-xs3)))+ys3; //mayor a
//               //Serial.println("puntos donde es circulo,x y y");
//               //Serial.print( f_y3);Serial.print(",");;Serial.print(f_yn3);Serial.print(",");;Serial.println(x_3);
//
//               if(xdf<=x_3+radio && xdf>=x_3-radio && ydf<=f_y3 && ydf>=f_yn3){
//             //Serial.println("recálculando...");
//                 randoms=random(0,1);
//
//                 if(varang[i18]>=posw){
//                    xdf=xs3;
//                    ydf=ys3+radio+20;
//                 }else{
//                       xdf=xs3;
//                       ydf=ys3-radio-20;
//                  }
//                Serial.print("nueva posicion x:"+String(xdf)+"\n");
//                Serial.print("nueva posicion y:"+String(ydf)+"\n");
//                  
//                                 //Serial.println("nuevas posiciones x y y");
//                //Serial.print(xdf);Serial.print(",");Serial.println(ydf);
//              }
//            }
//          }
//              
//           cont2=0;
//           
//              if(d1<=deteccionLimite){
//                cont2=cont2+1;
////                activacion=1;
////                float xn1=xs1-radio;
////                f_y=sqrt((radio*radio)-((xn1-xs1)*(xn1-xs1)))+ys1;//menor a
////               f_yn=-sqrt((radio*radio)-((xn1-xs1)*(xn1-xs1)))+ys1; //mayor a
////               Serial.println("puntos donde es circulo,x y y");
////               Serial.print(f_y);Serial.print(",");;Serial.print(f_yn);Serial.print(",");;Serial.println(xn1);
////
////               
////                xdf=xn1;
////                ydf=f_yn-20;
////                
////                if(dc>=ev){
////                  xn1=xn1+1;
////                  }
//                if(cont2>=50){
//                    ledcWrite(pwmChannel, dutyCycle);
//                    digitalWrite(motor1Pin1, HIGH);
//                    digitalWrite(motor1Pin2, LOW);
//                    ledcWrite(pwmChannel2, dutyCycle);
//                    digitalWrite(motor2Pin1, HIGH);
//                    digitalWrite(motor2Pin2, LOW);
//                  }
//                 xdf=xs1;
//                 ydf=ys1-radio-20;
//                }
//
//                              if(d2<=deteccionLimite){
//                                cont2=cont2+1;
////                activacion=1;
////                float xn1=xs1-radio;
////                f_y=sqrt((radio*radio)-((xn1-xs1)*(xn1-xs1)))+ys1;//menor a
////               f_yn=-sqrt((radio*radio)-((xn1-xs1)*(xn1-xs1)))+ys1; //mayor a
////               Serial.println("puntos donde es circulo,x y y");
////               Serial.print(f_y);Serial.print(",");;Serial.print(f_yn);Serial.print(",");;Serial.println(xn1);
////
////               
////                xdf=xn1;
////                ydf=f_yn-20;
////                
////                if(dc>=ev){
////                  xn1=xn1+1;
////                  }
//                       if(cont2>=50){
//                    ledcWrite(pwmChannel, dutyCycle);
//                    digitalWrite(motor1Pin1, HIGH);
//                    digitalWrite(motor1Pin2, LOW);
//                    ledcWrite(pwmChannel2, dutyCycle);
//                    digitalWrite(motor2Pin1, HIGH);
//                    digitalWrite(motor2Pin2, LOW);
//                  }
//                 xdf=xs2;
//                 ydf=ys2+radio+20;
//                }
//
//                              if(d3<=deteccionLimite){
//                                cont2=cont2+1;
////                activacion=1;
////                float xn1=xs1-radio;
////                f_y=sqrt((radio*radio)-((xn1-xs1)*(xn1-xs1)))+ys1;//menor a
////               f_yn=-sqrt((radio*radio)-((xn1-xs1)*(xn1-xs1)))+ys1; //mayor a
////               Serial.println("puntos donde es circulo,x y y");
////               Serial.print(f_y);Serial.print(",");;Serial.print(f_yn);Serial.print(",");;Serial.println(xn1);
////
////               
////                xdf=xn1;
////                ydf=f_yn-20;
////                
////                if(dc>=ev){
////                  xn1=xn1+1;
////                  }
//                       if(cont2>=50){
//                    ledcWrite(pwmChannel, dutyCycle);
//                    digitalWrite(motor1Pin1, HIGH);
//                    digitalWrite(motor1Pin2, LOW);
//                    ledcWrite(pwmChannel2, dutyCycle);
//                    digitalWrite(motor2Pin1, HIGH);
//                    digitalWrite(motor2Pin2, LOW);
//                  }
//                 randoms=random(0,1);
//
//                 if(randoms==1){
//                 xdf=xs3;
//                 ydf=ys3+radio+20;
//                 }else{
//                 xdf=xs3;
//                 ydf=ys3-radio-20;
//                  }
//                }
                
///////////////////////////conf_2
 

    //delay(1);
    delay(1);
  }
    wr=0;
  wl=0;
        stby = 0;
    ledcWrite(pwmChannel, wr);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel2, wl);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);

  Serial.println("la posicion actual en X es:" + String(posx));
  Serial.println("la posicion actual en Y es:" + String(posy));
  Serial.println("terminado pos:");
}


void calculapulsoD()
{
  int Lstated = digitalRead(Encoder_C1d);
  if ((Encoder_C1Lastd == LOW) && Lstated == HIGH)
  {
    int vald = digitalRead(Encoder_C2d);
    if (vald == LOW && direcciond)
    {
      direcciond = false; //Atrás
    }
    else if (vald == HIGH && !direcciond)
    {
      direcciond = true;  //Adelante
    }
  }
  Encoder_C1Lastd = Lstated;

  if (!direcciond)  pasoD++;
  else  pasoD--;
}

void calculapulsoI()
{
  int Lstate_i = digitalRead(Encoder_C1i);
  if ((Encoder_C1Lasti == LOW) && Lstate_i == HIGH)
  {
    int vali = digitalRead(Encoder_C2i);
    if (vali == LOW && direccioni)
    {
      direccioni = false; //Atrás
    }
    else if (vali == HIGH && !direccioni)
    {
      direccioni = true;  //Adelante
    }
  }
  Encoder_C1Lasti = Lstate_i;

  if (!direccioni)  pasoI++;
  else  pasoI--;
}
