#define NEGRO    0
#define ROJO     1
#define VERDE    2
#define AMARILLO 3
#define AZUL     4
#define MAGENTA  5
#define CYAN     6
#define BLANCO   7

#define PRIMER_AJUSTE 0
#define SIGUIENTE_AJUSTE 1
#define MEJOR_AJUSTE 2
#define PEOR_AJUSTE 3 


typedef int HCoche;

#define PARKING_MSG               100
#define PARKING_MSGSUB_APARCAR      1
#define PARKING_MSGSUB_DESAPARCAR   2

struct PARKING_mensajeBiblioteca
 {long   tipo;
  long   subtipo;
  HCoche hCoche;};
  
typedef int  (*TIPO_FUNCION_LLEGADA)              (HCoche hc);
typedef int  (*TIPO_FUNCION_SALIDA)               (HCoche hc);
typedef void (*TIPO_FUNCION_APARCAR_COMMIT)       (HCoche hc);
typedef void (*TIPO_FUNCION_PERMISO_AVANCE)       (HCoche hc);
typedef void (*TIPO_FUNCION_PERMISO_AVANCE_COMMIT)(HCoche hc);

int PARKING_inicio(int ret, TIPO_FUNCION_LLEGADA *f_llegadasP, int semAforos, int buzOn, int zona, int debug);
int    PARKING_aparcar(HCoche,void *datos,TIPO_FUNCION_APARCAR_COMMIT,
                                          TIPO_FUNCION_PERMISO_AVANCE,
                                          TIPO_FUNCION_PERMISO_AVANCE_COMMIT);
int PARKING_desaparcar(HCoche,void *datos,TIPO_FUNCION_PERMISO_AVANCE,
                                          TIPO_FUNCION_PERMISO_AVANCE_COMMIT);

int PARKING_fin(int normal);
int PARKING_simulaciOn();

int PARKING_getNUmero(HCoche);
int PARKING_getLongitud(HCoche);
int PARKING_getPosiciOnEnAcera(HCoche);
unsigned long PARKING_getTServ(HCoche);
int PARKING_getColor(HCoche);
void * PARKING_getDatos(HCoche);
int PARKING_getX(HCoche);
int PARKING_getY(HCoche);
int PARKING_getX2(HCoche);
int PARKING_getY2(HCoche);
int PARKING_getAlgoritmo(HCoche);
int PARKING_isAceraOcupada(int algoritmo,int pos);

int PARKING_getTamaNoMemoriaCompartida();
int PARKING_getNSemAforos();
