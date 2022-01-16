
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
 
/////////////////////////////////////////////////////////LIBRERIAS
#include "math.h"

/*Elige una opcion
 1=solo las posiciones deseadas
 2=genera  un camino basado en el RRT hasta las deseadas(RRT)
 3=RRT+suavizado
 4=RRT+suavizado+iterpolación
*/
//int opc = 1; //por el momento no es necesario 
//configuracion de posiciones deseadas en x y y
float xd = 200; //cm
float yd = 150; //cm

//posiciones inicial
float x_0 = 0;
float y_0 = 0;

//definir distancia D

float D = 20;

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
int randoms;
int opcion;
int hijos[101], cont = 0;
//////////////////////////////////////////////////////////////////////////////////////////
float alpha, betha, orientacion[101];

float psx[101], psy[101], itx[101], ity[101];
int NT = 10;
float mapaxf[100];
float mapayf[100];

/////////////////////////////////////////////////////////////////////////////////////CONTROL

float posx = 0, posy = 0, posw = 0,posd=0;
float xdf, ydf, ev, ew, v, w, wr, wl;
int cont4 = 0;

void setup() {
  Serial.begin(115200);
}


void loop() {

  while (j < (N + 1)) {

    //nuevos puntos de tamaño j
    float xn[j];
    float yn[j];

    //es necesario dejar un numero random de 0 a 1
    vu = random(0, 101);
    vu = vu / 100;
    vv = random(0, 101);
    vv = vv / 100;
    
    //punto aleatorio x y y
    xr = xl + (xu - xl) * (vu); //Xr
    yr = yl + (yu - yl) * (vv); //Yr
    
    //***IMPORTANTE PARA VER QUE SUCEDE NO OLVIDES DESCOMENTAR LAS LINEAS COMENTADAS.***
    
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
    for (int i = 0; i < j; i++) {

      
      //PRIMEROS PUNTOS DEL MAPA SON LAS INICIALES
      Mx[0] = x_0;
      My[0] = y_0;
      //POSICIONES ACTUALES
      xi[i] = Mx[i]; //Xi
      yi[i] = My[i]; //Yi


      //           Serial.print("posiciones xi y yi");
      //            Serial.println(" ");
      //          Serial.print(xi[i]); Serial.print(",");  Serial.print(yi[i]);
      //       Serial.println(" ");

      //sacamos la distancia respecto a posiciones xr, yr
      da = pow((xr - xi[i]), 2) + pow((yr - yi[i]), 2);
      d = sqrt(da);

      //                 Serial.print("distancia respecto a las posiciones xr y yr");
      //            Serial.println(" ");
      //          Serial.print(d);
      //       Serial.println(" ");
      //
      //       delay(2000);

      //si la disttancia es menor a la distancia minima entonces sustituye
      if (d < d_min) {
        d_min = d;
        i_min = i;//asignamos un valor "i"
        indice2[j] = i;//guardamos el valor en posicion j de  el grafo source/hijo/raiz

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

    //sacamos vecotr unitarios de x,y
    vx[j] = xr - xc[j];
    vy[j] = yr - yc[j];
    v2 = pow((xr - xc[j]), 2) + pow((yr - yc[j]), 2);
    v2 = sqrt(v2);
    vx[j] = vx[j] / v2; //vector unitario x
    vy[j] = vy[j] / v2; //vector unitario y
    
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

    //posiciones nuevas guardadas
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


   //distancias de xn,yn respecto a xd,yd(deseadas)
    nor = pow((xd - xn[j]), 2) + pow((yd - yn[j]), 2);
    nor = sqrt(nor);
    
    //        ////////////////poner una condicion  que se acerrque sino reinicciar
    //       Serial.println("distancia de posicion xn y yn respecto a  xd,yd");
    //       Serial.print(nor);
    //       Serial.println(" ");
    //       delay(2000);

    Serial.println(" ");

    Serial.print("ciclo");
    Serial.print("[");
    Serial.print(j);
    Serial.print("]");


    Serial.println("");

    if (nor < D) { //si esta lo suficientemente cercano rompo el ciclo

      //y agregamos al grafos y listas de padres e hijos
      Mx[j + 1] = xd;
      My[j + 1] = yd;

      Serial.print("[posición x:]");

      for (int j1 = 0; j1 < j + 1; j1++) {
        Serial.print("[");
        Serial.print(j1);
        Serial.print("]");
        Serial.print("[");
        Serial.print(Mx[j1]);
        Serial.print("]");

      }
      Serial.print("[");
      Serial.print(j + 1);
      Serial.print("]");
      Serial.print("[");
      Serial.print( Mx[j + 1]);
      Serial.print("]");

      Serial.println(" ");

      Serial.print("[posición y:]");

      for (int i2 = 0; i2 < j + 1; i2++) {
        Serial.print("[");
        Serial.print(i2);
        Serial.print("]");
        Serial.print("[");
        Serial.print(My[i2]);
        Serial.print("]");
      }

      Serial.print("[");
      Serial.print(j + 1);
      Serial.print("]");
      Serial.print("[");
      Serial.print( My[j + 1]);
      Serial.print("]");
      Serial.println(" ");

      Serial.println(" ");

      Serial.print("sources");

      for (int i4 = 0; i4 < j + 1; i4++) {
        Serial.print("[");
        Serial.print(indice2[i4]);
        Serial.print("]");

      }
      Serial.println(" ");

      Serial.print("targets");
      for (int i6 = 0; i6 < j + 1; i6++) {
        target2[j] = i6;
        Serial.print("[");
        Serial.print(target2[j]);
        Serial.print("]");
      }

      Serial.println(" ");


      /////////////////////////////////comparar con boleeanos
      //es necesario compararlos asi ya que por alguna razon los arreglos no se comparan
      //sse empieza desde el ultimo hasta el primero de atras hacia delante
      i11 = j;
      for (i10 = j; i10 >= 0; i10--) {
        target2[j] = i10;

        int test1 = indice2[i11];
        int test2 = target2[j];
        //
        //Serial.print("resultado");
        if ( test1 == test2) { //comparamos
          
          
          //     Serial.println(" ");

          //            Serial.print("[");
          //       Serial.print(resultado);
          //     Serial.print("]");
          
          //creamos un contador para definir el tamaño de algunos vectores
          cont = cont + 1;
          
          hijos[cont + 1] = target2[j];//asignamos en posiciones los hijos
          padres[cont] = indice2[i11];//asignamos en posiciones los padres
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

      Serial.println(" ");
      Serial.print("padres");
      for (int i12 = cont; i12 > 0; i12--) {

        Serial.print("[");
        Serial.print(padres[i12]);
        Serial.print("]");

      }

      Serial.println(" ");
      Serial.print("hijos");
      for (int i13 = cont; i13 > 0; i13--) {
        hijos[1] = j;
        Serial.print("[");
        Serial.print(hijos[i13]);
        Serial.print("]");

      }
      Serial.println(" ");
      Serial.println(" ");
      Serial.print("padres distancias");
      for (int i14 = cont; i14 > 0; i14--) {

        Serial.print("[");
        Serial.print(mapax1[i14]);
        Serial.print("]");
        Serial.print("[");
        Serial.print(mapay1[i14]);
        Serial.print("]");
        Serial.print(",");
      }

      Serial.println(" ");
      Serial.print("hijos distanacias");
      for (int i15 = cont; i15 > 0; i15--) {
        mapax2[1] = xd;
        mapay2[1] = yd;
        Serial.print("[");
        Serial.print(mapax2[i15]);
        Serial.print("]");
        Serial.print("[");
        Serial.print(mapay2[i15]);
        Serial.print("]");
        Serial.print(",");
      }
      Serial.println(" ");

      Serial.println(" ");

      //creamos otro mapa final donde guardaremos  las posiciones a ir
      mapaxf[cont];
      mapayf[cont];
      for (int i19 = 0; i19 <= cont; i19++) {

        mapaxf[i19] = mapax1[cont - i19];
        mapayf[i19] = mapay1[cont - i19];
      }
      Serial.println(" ");



      Serial.print("mapeado x,y");
      for (int i20 = 0; i20 <= cont; i20++) {
        mapaxf[cont] = xd;
        mapayf[cont] = yd;

        Serial.print("[");
        Serial.print(mapaxf[i20]);
        Serial.print("]");
        Serial.print("[");
        Serial.print(mapayf[i20]);
        Serial.print("]");
        Serial.print(",");
      }
      Serial.println(" ");

      /////////////////////////////////////////SUAVIZADO DE TRAYECTORIA

      for (int i16 = 0; i16 <= cont; i16++) {
        psx[i16] = mapaxf[i16];
        psy[i16] = mapayf[i16];
        
        //puedes cambiar los parametros alpha y betha
        alpha = 0.01;
        betha = 0.02;
        psx[i16] = psx[i16] + alpha * (mapax1[i16] - psx[i16]) + betha * (psx[i16 + 1] + psx[i16 - 1] - 2 * psx[i16]);
        psy[i16] = psy[i16] + alpha * (mapay1[i16] - psy[i16]) + betha * (psy[i16 + 1] + psy[i16 - 1] - 2 * psy[i16]);
        Serial.println("mapa ");
        Serial.print("[");
        Serial.print(mapaxf[i16]);
        Serial.print("]");  Serial.print("[");
        Serial.print(mapayf[i16]);
        Serial.print("]");
        Serial.print(",numero:");
        Serial.print(i16);
        Serial.println(" ");
        Serial.println("suavizado ");
        Serial.print(psx[i16]); Serial.print(","); Serial.println(psy[i16]);




      }
      Serial.println(" ");

      /////////////////////////////////ITERPOLACIÓN O AGREGAR PUNTOS

      Serial.print("iterpolacion");
      float alphai;
      for (int i17 = 0; i17 <= cont; i17++) {
        cont4 = i17 * 3;
        for (int i21 = 1; i21 < 4; i21++) {
          alphai = i21 / 4.0;
          // alphai=0.05;

          itx[cont4 + i21] = psx[i17 - 1] + alphai * (psx[i17] - psx[i17 - 1]);
          ity[cont4 + i21] = psy[i17 - 1] + alphai * (psy[i17] - psy[i17 - 1]);
          //                       Serial.println(" ");       Serial.print("alpha[");
          //       Serial.print(alphai);
          //     Serial.print("]");

          Serial.println(" ");
          Serial.print(itx[cont4 + i21]); Serial.print(","); Serial.print(ity[cont4 + i21]);
        }

      }
      Serial.println(" ");

      Serial.println(" ");

      //               Serial.print("Source y target");   Serial.println(" ");
      //       Serial.print(S[j]);Serial.print(",");Serial.println(T[j]);
      //       Serial.println(" ");
      //       delay(2000);

      j = j + N + 2;

      resultado = true;
      break;

    }
    j=j+1; 
  }
     if (resultado == false ) { //sirve como señal si no se ha llegado a ser verdadero  que se ha llegado a una trayectoria  se vuelve a reiniciar los ciclos
    j = 0;
    cont = -1;
  }
}
