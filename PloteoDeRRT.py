import serial
import time
import numpy as np
import cv2

mapax=np.zeros((1,100))
mapay=np.zeros((1,100))

suavizadox=np.zeros((1,100))
suavizadoy=np.zeros((1,100))

iterpolacionx=np.zeros((1,100))
iterpolaciony=np.zeros((1,100))

imagen = 255*np.ones((800,1200,3),dtype=np.uint8)

i=0
i2=0
i3=0
i4=0
i5=0
i6=0
ij=0
ik=0
ik2=0
ij2=0
etiqueta=0
registro=0
ar=0
h=0
g=0

posx=0
posy=0
posw=0

nposx1=0
nposy1=0

xs1=0
xs2=0
xs3=0
ys1=0
ys2=0
ys3=0
activacion=0
activacion1=0
conta=0
cont=0
ig=0

ar=serial.Serial("COM6",115200)

time.sleep(2)

    
while conta<=i5:
     
     entrada=ar.readline().decode('ascii')
     #cv2.imshow('imagen',imagen)
     registro=entrada.index(":")
     etiqueta=entrada[:registro]
     salida=entrada[registro+1:]
     

     if etiqueta=='posicion x':
         i=i+1
         salida=float(salida)
         mapax[0][i]=salida
         
     elif etiqueta=='posicion y':
         i2=i2+1 
         salida=float(salida)
         mapay[0][i2]=salida

     elif etiqueta=='suavizado x':
         i3=i3+1
         salida=float(salida)
         suavizadox[0][i3]=salida

     elif etiqueta=='suavizado y':
         i4=i4+1
         salida=float(salida)
         suavizadoy[0][i4]=salida

     elif etiqueta=='iterpolacion x':
         conta=conta+1
         i5=i5+1
         salida=float(salida)
         iterpolacionx[0][i5]=salida

     elif etiqueta=='iterpolacion y':
         i6=i6+1
         salida=float(salida)
         iterpolaciony[0][i6]=salida

     elif etiqueta=='terminado RRT':
         conta=i5+2
         break
 



mapaLista=[0 for ia in range(i+2)]

suavizadoLista=[0 for ib in range(i3+2)]

iterpolacionLista=[0 for ic in range(i5+3)]

#iterpolacionLista[i5+1]=(600+200,400-150)

#mapa
for i1a in range(i+1):
    a=mapax[0][i1a]
    a=int(a)
    b=mapay[0][i1a]
    b=int(b)
    mapaLista[i1a]=(600+a,400-b)#solo para plotear

        #suavizado
for i1b in range(i3+1):
    c=suavizadox[0][i1b]
    c=int(c)
    d=suavizadoy[0][i1b]
    d=int(d)
    suavizadoLista[i1b]=(600+c,400-d)#solo para plotear

        #iterpolacion
for i1c in range(i5+1):
    e=iterpolacionx[0][i1c]
    e=int(e)
    f=iterpolaciony[0][i1c]
    f=int(f)
    iterpolacionLista[i1c]=(600+e,400-f)#solo para plotear

cv2.circle(imagen,(600+250,400-150),10,(0,0,255),-1) #ploetamos  en un circulo rojo la posiciones deseadas
cv2.circle(imagen,(600+0,400-0),10,(0,0,255),-1)# puntos iniciales    

for ic in range(i+1):
        #mapa
      
    cv2.circle(imagen,mapaLista[ic],5,(255,0,0),1)
        #suavizado
    
    cv2.circle(imagen,suavizadoLista[ic],5,(0,255,0),1)        
    
    cv2.imshow('imagen',imagen)




for ica in range(i5+1):
    
    cv2.circle(imagen,(iterpolacionLista[ica]),5,(0,128,255),1)            
    
    cv2.imshow('imagen',imagen)

     


cv2.imshow('imagen',imagen)
cv2.waitKey(0) 
cv2.destroyAllWindows()

while True:     

     entrada=ar.readline().decode('ascii')

     registro=entrada.index(":")
     etiqueta=entrada[:registro]
     salida=entrada[registro+1:]

     posicionesLista=[600+posx,400-posy]
    
     if etiqueta=='pos x':
         salida=float(salida)
         posx=salida
         posx=int(salida)

     elif etiqueta=='pos y':
         salida=float(salida)
         posy=salida
         posy=int(posy)

     elif etiqueta=='pos w':
         salida=float(salida)
         posw=salida
         posw=int(posw)

     elif etiqueta=='xs1':
         salida=float(salida)
         xs1=salida
         xs1=int(xs1)

     elif etiqueta=='ys1':
         salida=float(salida)
         ys1=salida
         ys1=int(ys1)

     elif etiqueta=='xs2':
         salida=float(salida)
         xs2=salida
         xs2=int(xs2)

     elif etiqueta=='ys2':
         salida=float(salida)
         ys2=salida
         ys2=int(ys2) 

     elif etiqueta=='xs3':
         salida=float(salida)
         xs3=salida
         xs3=int(xs3)

     elif etiqueta=='ys3':
         salida=float(salida)
         ys3=salida
         ys3=int(ys3)

     elif etiqueta=='nueva posicion x':
         salida=float(salida)
         nposx1=salida
         nposx1=int(nposx1)

     elif etiqueta=='nueva posicion y':
         salida=float(salida)
         nposy1=salida
         nposy1=int(nposy1)           

     
     elif etiqueta=='terminado pos':
         break
     
     #if class_ind==1:
        # cv2.circle(imagen,(600+posx,400-posy),20,(0,102,204),2)

    
     nuevapos=(600+nposx1,400-nposy1)
        #posesiones actuales
     cv2.circle(imagen,posicionesLista,7,(0,0,0),-1)
       
       #obstáculos
     cv2.circle(imagen,(600+xs1,400-ys1),10,(0,0,255),3)
     cv2.circle(imagen,(600+xs2,400-ys2),10,(0,0,255),3)
     cv2.circle(imagen,(600+xs3,400-ys3),10,(0,0,255),3)

     #nuevas posciones
     cv2.circle(imagen,nuevapos,7,(0,255,0),-1)
     cv2.circle(imagen,(600+posx,400-posy),10,(0,128,255),1) 
            #posiciones angulares
     #cv2.line(imagen,anguloLista1,anguloLista2,(0,0,0),4)                          
     cv2.imshow('imagen',imagen)
     if cv2.waitKey(1) & 0xFF==ord('q') :
         break


    
ar.close()###########################################################<---
cv2.destroyAllWindows()  
