/*************************************************
 *  Source: parking.c                            *
 *                                               *
 *  @author: Luis Blázquez Miñambres y Samuel    *
 *           Gómez Sánchez                       *
 *                                               *
 *  @version: 1034.2                             *
 *                                               *
 *  @date: 25/04/2018                            *
 *                                               *
 *  @brief: Programa de simulación de gestión de *
 *          procesos en un sistema operativo con *
 *          interfaz gráfica que simula coches   *
 *          aparcando. Hermoso castigo.          *
 *************************************************/

//#define PRIMER_AJUSTE_OFF
//#define SIGUIENTE_AJUSTE_OFF
//#define MEJOR_AJUSTE_OFF
//#define PEOR_AJUSTE_OFF


/************************/
/*    Header files      */
/************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <errno.h>
#include "parking.h"

/**************************/
/*   Headers manuales     */
/**************************/

#define PRIOR_APARCAR           0
#define PRIOR_DESAPARCAR        1

#define N_MSG_APARCAR       0
#define N_MSG_DESAPARCAR    1

#define ROAD_LENGTH         80
#define EMPTY               0
#define OCCUPIED            1
#define RESERVED            2

#define N_ROADS             3
#define LOTS                0
#define STRIPE              1
#define LANE                2

#define N_PIDS              3
#define MAIN_ADDR           0
#define TIMER_ADDR          1
#define MANAGER_ADDR        2
#define CHOFER_ADDR         3
#define N_AJUSTES           4

#define create_msg() msgget(IPC_PRIVATE,  0777 | IPC_CREAT)
#define create_shmem(shmsize) shmget(IPC_PRIVATE, (shmsize), IPC_CREAT | 0777)

#define N_ADD_SEMS                      6
#define SEM_MSG                         0
#define SEM_MSG_COUNTERS                1
#define SEM_ORDEN_COCHES(ajuste)        (2 + (ajuste))

#define TIPO_MSG_PERMISO_AVANCE(ajuste, posicion)\
        (3000 + 100*(ajuste) + (posicion))

struct shm_rsrc {
    pid_t * pid;
    int     n_children;
    int     road[N_AJUSTES][N_ROADS][ROAD_LENGTH];
    int     mensajes[2]; // El primero indica mensajes de aparcar,
                         // y el segundo de desaparcar
};

struct ipc_rsrc {
    int                 nsems;
    int                 sem_0;
    int                 semid;
    int                 msgid;
    int                 shmid;
    size_t              shmsize, lib_shmsize;
    char *              lib_shmaddr;
    struct shm_rsrc *   shmaddr;
};

struct msg_subtipo{
    long subtipo;
    HCoche hCoche;
};

struct msg_permiso_avance {
    long alg_pos_id;
};

/* Prototypes */
int short_help_msg(void);
int help_msg(void);
void dummy (void);
int test_args(int argc, char * argv[]);
int invalid_option_msg(char * opt);
void cleanup(void);
void * calculate_shmaddr(struct ipc_rsrc * ipc);
void * calculate_pid_addr(struct ipc_rsrc * ipc);
int init_ipc(struct ipc_rsrc * ipc, int n_chofer);
int init_shm(struct shm_rsrc * shm_addr);

int create_binsem(int semnum);
int semwait(int semid, int semnum);
int semwait_0(int semid, int semnum);
int semwait_val(int semid, int semnum, int val);
int semsig(int semid, int semnum);
int semsig_val(int semid, int semnum, int val);
int set_semval(int semid, int semnum, int val);
int get_semval(int semid, int semnum);
// create_msg() implementada como macro
void aparcar_commit(HCoche hc);
void permiso_avance_commit(HCoche hc);
void permiso_avance(HCoche hc);

/*Manejadoras*/
void sigint_hdlr(int signal);
void sigalrm_hdlr(int signal);
void sigusr1_hdlr(int signal);

/* Funciones de llegada */
int primer_ajuste (HCoche);
int siguiente_ajuste(HCoche);
int mejor_ajuste(HCoche);
int peor_ajuste(HCoche);
int siguiente_ajuste_primer_hueco(int last_pos);
int siguiente_ajuste_ajusta_primer(int c_length, int a, int b);

/************************/
/*  Variables globales  */
/************************/
struct ipc_rsrc ipc;
int sigalrm_received = 0;
int sigusr1_received = 0;

/*****************************************************************************/
/************************************* MAIN **********************************/
/*****************************************************************************/

int main(int argc, char * argv[])
{
    int i,j, tmp;
    struct sigaction sa; 
    sigset_t mask;

    /* Array de funciones de rellamada para la funcion
     * parking inicio
     */
    TIPO_FUNCION_LLEGADA funciones_llegada[4] = {  primer_ajuste,
                                                   siguiente_ajuste,
                                                   mejor_ajuste,
                                                   peor_ajuste};
    /* Para almacenar los argumentos por CLI tras
     * testear que son apropiados
     *
     * Prioridad = 0 -> PA
     * Prioridad = 1 -> PD
     */
    int rapidez, n_chofer, debug_flag, prioridad;
    /* Comprobamos los argumentos recibidos; si son invalidos, no se 
     * continua la ejecucion
     */
    tmp = test_args(argc, argv);
    if (tmp == -1){
        return -1;
    } else {
        rapidez = atoi(argv[1]); 
        n_chofer = atoi(argv[2]);
        if (tmp & 0x4) // Testeamos a nivel de bit. Ver test_args
            debug_flag = 1;
        else
            debug_flag = 0;
        if (tmp & 0x2)
            prioridad = PRIOR_APARCAR;
        else // Por defecto, si no se indica, prioridad PD
            prioridad = PRIOR_DESAPARCAR;
    }

    /* Inicializamos variables y recursos */
    // Cambiamos comportamiento de SIGINT
    if (sigemptyset(&mask) == -1){
        perror("main: sigemptyset");
        return -1;
    }

    sa.sa_handler = sigint_hdlr;
    sa.sa_mask = mask;
    sa.sa_flags = 0;

    if(sigaction(SIGINT, &sa, NULL) == -1){
        perror("main: sigaction");
        return -1;
    }

    /* Registramos la funcion de limpieza */
    atexit(cleanup);

    /* Obtenemos los recursos de IPC */
    if (init_ipc(&ipc, n_chofer) == -1){
        fprintf(stderr, "%s\n", "init_ipc: Error al crear los recursos IPC");
        exit(-1);
    }

    /* Construye los recursos iniciales de la simulacion */
    PARKING_inicio( rapidez,
                    funciones_llegada,
                    ipc.semid,
                    ipc.msgid,
                    ipc.shmid,
                    debug_flag);


    /* El proceso principal escribe su PID en la memoria compartida para
     * que el resto pueda comunicarse con el
     */
    ipc.shmaddr->pid[MAIN_ADDR] = getpid();

    /* Creamos los procesos auxiliares */
    switch(fork()){
    case -1:
        perror("main: fork: cronometro");
        exit(-1);
        break; 
    case 0: // Proceso cronómetro
	    atexit(dummy);
        ipc.shmaddr->pid[TIMER_ADDR] = getpid();

        // Cambiamos comportamiento de SIGALRM
        sa.sa_handler = sigalrm_hdlr;
        sa.sa_mask = mask;
        sa.sa_flags = 0;

        if(sigaction(SIGALRM, &sa, NULL) == -1){
            perror("cronometro: sigaction");
            if (kill(ipc.shmaddr->pid[MAIN_ADDR], SIGINT) == -1){
                perror("cronometro: kill");
                return -1;
            }
            return -1;
        }

        // Cambiamos comportamiento de SIGUSR1

        sa.sa_handler = sigusr1_hdlr;
        sa.sa_mask = mask;
        sa.sa_flags = 0;

        if(sigaction(SIGUSR1, &sa, NULL) == -1){
            perror("cronometro: sigaction");
            if (kill(ipc.shmaddr->pid[MAIN_ADDR], SIGINT) == -1){
                perror("cronometro: kill");
                return -1;
            }
            return -1;
        }

        alarm(30);

        while (!sigalrm_received && !sigusr1_received)
            sigsuspend(&mask);

        if (sigusr1_received){
            PARKING_fin(0);
        } else {
            PARKING_fin(1);
        }

        return 0;
    }

    switch(fork()){
    case -1:
        perror("main: fork: gestor");
        exit(-1);
        break; 
    case 0: // Proceso gestor
	    atexit(dummy);
        ipc.shmaddr->pid[MANAGER_ADDR] = getpid();
        struct PARKING_mensajeBiblioteca msgp;
        struct msg_subtipo submsg;

        while (1){

            /* Gestor recive el mensaje para aparcar o desaparcar por parte de 
             * la biblioteca de tipo PARKING_MSG
             */
            ssize_t rvalue = msgrcv(ipc.msgid,
                                    &msgp,
                                    sizeof(struct PARKING_mensajeBiblioteca),
                                    PARKING_MSG,
                                    0);
            if (rvalue == -1){

                perror("gestor: msgrcv");
                if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                    perror("gestor: kill");
                    return -1;
                }
                return -1;
            }

            submsg.subtipo = msgp.subtipo;
            submsg.hCoche = msgp.hCoche;

/* Semáforos representados:
*    - ipc.sem_0 + SEM_MSG:   primer semáforo de uso propio.
*
*                   Coordina a los distintos choferes, de manera 
*                   que cuando hay un mensaje (o mas) un chofer
*                   se "reserva" un mensaje, y a la seccion de 
*                   codigo siguiente solo pueden entrar tantos 
*                   choferes como mensajes.
*
*                   Notese que los choferes simplemente lo 
*                   decrementan, y el gestor es quien lo incrementa
*                   (se usa como un contador a la vez que sincroniza)
*
*	  - ipc.sem_0 + SEM_MSG_COUNTERS: semáforo binario para evitar 
*                                     que las variables de memoria
*                                     compartida sean editadas por
*                                     más de un proceso a la vez.
*
*/

            if (semwait(ipc.semid, ipc.sem_0 + SEM_MSG_COUNTERS) == -1){
                perror("gestor: semwait");
                if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                    perror("gestor: kill");
                    return -1;
                }
                return -1;
            }

                if (submsg.subtipo == PARKING_MSGSUB_APARCAR){
                    ++(ipc.shmaddr->mensajes[N_MSG_APARCAR]);
                } else {
                    ++(ipc.shmaddr->mensajes[N_MSG_DESAPARCAR]);
                }
                
                if (semsig(ipc.semid, ipc.sem_0 + SEM_MSG) == -1){
                    perror("gestor: semsig");
                    if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                        perror("gestor: kill");
                        return -1;
                    }
                    return -1;
                }

                if (msgsnd(ipc.msgid, &submsg, sizeof(struct msg_subtipo), 0)
                == -1){
                    perror("gestor: msgsnd");
                    if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                        perror("gestor: kill");
                        return -1;
                    }
                    return -1;
                }

            if (semsig(ipc.semid, ipc.sem_0 + SEM_MSG_COUNTERS) == -1){
                perror("gestor: semsig");
                if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                    perror("gestor: kill");
                    return -1;
                }
                return -1;
            }
        }

        break;
    }

    for (i = 0; i < n_chofer; ++i){
        switch(fork()){
        case -1:
            perror("fork");
            exit(-1);
            break; 
        case 0: // Proceso chófer
	    atexit(dummy);

        ipc.shmaddr->pid[CHOFER_ADDR + i] = getpid();

        long msg_received;
        struct msg_subtipo msgp;

            while (1){

                if (semwait(ipc.semid, ipc.sem_0 + SEM_MSG)== -1){
                    perror("chofer: semwait");
                    if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                        perror("chofer: kill");
                    }
                    return -1;
                }

                if (semwait(ipc.semid, ipc.sem_0 + SEM_MSG_COUNTERS) == -1){
                    perror("chofer: semwait");
                    if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                        perror("chofer: kill");
                    }
                    return -1;
                }

                    if (prioridad == PRIOR_APARCAR){
                        if (ipc.shmaddr->mensajes[N_MSG_APARCAR] > 0){
                            msg_received = PARKING_MSGSUB_APARCAR;
                        } else {
                            msg_received = PARKING_MSGSUB_DESAPARCAR; 
                        }
                    } else if (prioridad == PRIOR_DESAPARCAR){
                        if (ipc.shmaddr->mensajes[N_MSG_DESAPARCAR] > 0){
                            msg_received = PARKING_MSGSUB_DESAPARCAR;
                        } else {
                            msg_received = PARKING_MSGSUB_APARCAR; 
                        }
                    } else { //FIFO
                        /* Recibiremos el primer mensaje de la 
                         * cola con valor menor o igual a
                         * PARKING_MSGSUB_DESAPARCAR
                         */
                        msg_received = (-1) * PARKING_MSGSUB_DESAPARCAR;
                    }

                    ssize_t rvalue = msgrcv(ipc.msgid,
                                            &msgp,
                                            sizeof(struct msg_subtipo),
                                            msg_received,
                                            0);

                    if (rvalue == -1){
                        perror("chofer: msgrcv");
                        if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                            perror("chofer: kill");
                            return -1;
                        }
                        return -1;
                    }

                    if (msgp.subtipo == PARKING_MSGSUB_APARCAR){
                        --(ipc.shmaddr->mensajes[N_MSG_APARCAR]);
                    } else {
                        --(ipc.shmaddr->mensajes[N_MSG_DESAPARCAR]);
                    }

                if (semsig(ipc.semid, ipc.sem_0 + SEM_MSG_COUNTERS) == -1){
                    perror("chofer: semsig");
                    if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                        perror("chofer: kill");
                        return -1;
                    }
                    return -1;
                }
                
                if (msgp.subtipo == PARKING_MSGSUB_APARCAR){

                    switch(PARKING_getAlgoritmo(msgp.hCoche)){
                    case PRIMER_AJUSTE:
            		    if (semwait_val(ipc.semid, 
                                ipc.sem_0 + SEM_ORDEN_COCHES(PRIMER_AJUSTE),
                                PARKING_getNUmero(msgp.hCoche)) == -1){

                            perror("chofer: semwait_val");
                            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                                perror("chofer: kill");
                                return -1;
                            }
                            return -1;

                        }
                        break;
                    case SIGUIENTE_AJUSTE:
            		    if (semwait_val(ipc.semid, 
                                ipc.sem_0 + SEM_ORDEN_COCHES(SIGUIENTE_AJUSTE),
                                PARKING_getNUmero(msgp.hCoche)) == -1){

                            perror("chofer: semwait_val");
                            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                                perror("chofer: kill");
                                return -1;
                            }
                            return -1;

                        }
                        break;
                    case MEJOR_AJUSTE:
            	        if (semwait_val(ipc.semid, 
                                ipc.sem_0 + SEM_ORDEN_COCHES(MEJOR_AJUSTE),
                                PARKING_getNUmero(msgp.hCoche)) == -1){

                            perror("chofer: semwait_val");
                            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                                perror("chofer: kill");
                                return -1;
                            }
                            return -1;

                        }
                        break;
                    case PEOR_AJUSTE:
            		    if (semwait_val(
                                ipc.semid, 
                                ipc.sem_0 + SEM_ORDEN_COCHES(PEOR_AJUSTE),
                                PARKING_getNUmero(msgp.hCoche)) == -1){

                            perror("chofer: semwait_val");
                            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                                perror("chofer: kill");
                                return -1;
                            }
                            return -1;

                        }
                        break;
                    default:
                        fprintf(
                            stderr, "%s\n", 
                            "Proceso chófer: numero de algoritmo no válido");
                        break;
                    }

                    PARKING_aparcar(msgp.hCoche,
                                    NULL,
                                    aparcar_commit,
                                    permiso_avance,
                                    permiso_avance_commit);

                } else {
                    PARKING_desaparcar(msgp.hCoche,
                                       NULL,
                                       permiso_avance,
                                       permiso_avance_commit);
                }
            }

            break;
        }
    }

    PARKING_simulaciOn();

    return 0;
}

void dummy (void){}

/*******************************
 *  sigint_hdlr                * 
 *******************************
 *                             *
 * Manejadora para SIGINT      *
 *******************************/
void sigint_hdlr(int signal)
{
    exit(0);
}

/*******************************
 *  sigalrm_hdlr               * 
 *******************************
 *                             *
 * Manejadora para SIGALRM     *
 *******************************/
void sigalrm_hdlr(int signal)
{
    sigalrm_received = 1;
}

/*******************************
 *  sigusr1_hdlr               * 
 *******************************
 *                             *
 * Manejadora para SIGALRM     *
 *******************************/
void sigusr1_hdlr(int signal)
{
    sigusr1_received = 1;
}

/************************************
 *  cleanup                         * 
 ************************************
 *                                  *
 *  Funcion para registrar con      *
 *  atexit(): elimina los recursos  *
 *  de IPC asi como los procesos    *
 *  hijo al terminar la ejecucion   *
 *  tanto si la finalizacion es     *
 *  normal como si se debe a un     *
 *  error.                          *
 ************************************/
void cleanup(void)
{
    int i;
    int counter;
    static int called = 0;

    /* Como la memoria compartida se inicializa a -1, si el valor de
     * los datos es -1, significa que no se crearon, y no se deben
     * intentar eliminar
     */

    counter = 0;
    if (!called){
        for(i = 1; i <= ipc.shmaddr->n_children; i++){
            if (kill(ipc.shmaddr->pid[i], SIGTERM) == -1){
                perror("cleanup: kill");
                counter++;
            }
        }

        // __DEBUG__ 
        // Aun no me he asegurado de que esto funcione bien (estos wait)
        for(i = 1; i <= ipc.shmaddr->n_children - counter; i++){
            if (wait(NULL) == -1){
                perror("cleanup: wait");
            }
        }

        if (ipc.semid != -1){
            if (semctl(ipc.semid, 0, IPC_RMID) == -1){
                perror("cleanup: semctl");
            } 
        }


        if (ipc.msgid != -1){
            if (msgctl(ipc.msgid, IPC_RMID, NULL) == -1){
                perror("cleanup: msgctl");
            }
        }

        if (ipc.shmid != -1){
            if (shmdt(ipc.lib_shmaddr) == -1){ 
                perror("cleanup: shmdt");
            }
        }

        if (ipc.shmid != -1){
            if (shmctl(ipc.shmid, IPC_RMID, 0) == -1){
                perror("cleanup: shmctl");
            }
        }

        called = 1;
    }
    
}

/***************************************
 *  test_args                          * 
 ***************************************
 *                                     *
 *  Comprueba que los argumentos       *
 *  recibidos por CLI tienen el        *
 *  formato apropiado.                 *
 *                                     *
 *  Return: un numero que a nivel      *
 *          de bit indica las opciones *
 *          si tiene exito, -1 si hay  *
 *          algun error.               *
 ***************************************/
int test_args(int argc, char * argv[])
{
    int i, j; // Indices

    /* Para almacenar las opciones D, PA y PD bitwise.
     * Así:
     *      100 -> D
     *      010 -> PA
     *      001 -> PD
     *      101 -> D y PD
     * etc.
     */
    int options = 0;

    if (argc < 2){

        invalid_option_msg("None");
        return -1;

    } else {

        if (argc == 2) {

            if (!strcmp(argv[1], "--help"))
                help_msg();
            else
                short_help_msg();

            return -1;

        } else if (argc >= 3){

            // Comprobamos si los primeros argumentos son numeros
            i = 0;
            while (argv[1][i] != '\0'){
                if (!isdigit(argv[1][i]))
                    return -1;
                ++i;
            }

            i = 0;
            while (argv[2][i] != '\0'){
                if (!isdigit(argv[2][i]))
                    return -1;
                ++i;
            }

            /* Comprobamos que el resto de argumentos no producen combinaciones
             * no validas:
             *      - No se repiten opciones
             *      - No se dan PA y PD a la vez
             */
            for (i = 3; i < argc; ++i){

                j = 0;
                while (argv[i][j] != '\0'){
                    argv[i][j] = toupper(argv[i][j]);
                    ++j;
                }

                if (!strcmp(argv[i], "D")){
                    if (options & 0x4){ // Test bitwise
                        invalid_option_msg(argv[i]);
                        return -1;
                    } else {
                        options |= 0x4; // Set bitwise
                    }
                } else if (!strcmp(argv[i], "PA")){
                    if ((options & 0x1) || (options & 0x2)) {
                        invalid_option_msg(argv[i]);
                        return -1;
                    } else {
                        options |= 0x2;
                    }
                } else if (!strcmp(argv[i], "PD")){
                    if ((options & 0x2) || (options & 0x1)) {
                        invalid_option_msg(argv[i]);
                        return -1;
                    } else {
                        options |= 0x1;
                    }
                } else {
                    invalid_option_msg(argv[i]);
                    return -1;
                }
            }
        }

        return options;
    }
}

/************************************
 *  invalid_option_msg              * 
 ************************************
 *                                  *
 *  Imprime un mensaje de error     *
 ************************************/
int invalid_option_msg(char * opt)
{
    char msg[1000] = "parking: invalid option -- '%s'\n\
Try 'parking --help' for more information.\n"; 

    return printf(msg, opt);
}

/************************************
 *  short_help_msg                  * 
 ************************************
 *                                  *
 *  Imprime un mensaje de ayuda     *
 *  corto                           *
 ************************************/
int short_help_msg(void)
{

    char msg[1000] = "Usage: parking SPEED N [D] [PA | PD]\n\
Try 'parking --help' for more information.\n"; 
                         
    return printf("%s", msg);

}

/************************************
 *  help_msg                        * 
 ************************************
 *                                  *
 *  Imprime un mensaje de ayuda     *
 ************************************/
int help_msg(void)
{

    char msg[1000] = "Usage: parking SPEED N [D] [PA | PD]\n\
Simulates a process  allocation  system with  an  interface  that  emulates a\n\
road with cars which must be parked in a ordered fashion.\n\
    SPEED               Controls the speed of events. 0 is the  higher speed,\n\
                        and subsequent  incresing values, up to INT_MAX, slow\n\
                        down the simulation.\n\
    N                   N is an integer that sets up the number of allocating\n\
                        processes ('chauffeurs') which must be generated.\n\
    D                   A 'D' can be used  to  produce debugging  information\n\
                        about the simulation, which will be output on stderr.\n\
    PA                  Selects priority for parking over unparking.\n\
    PD                  Selects priority for unparking over parking.\n";
                         
    return printf("%s\n", msg);

}

/***************************************
 *  create_binsem                      * 
 ***************************************
 *                                     *
 *  Crea un array de tantos semaforos  *
 *  de System V como se indica en su   *
 *  argumento. Son semaforos binarios  *
 *                                     *
 *  Return: el identificador del array *
 *          de semaforos en el sistema *
 *          si tiene exito, -1 si      *
 *          falla                      *
 *                                     *
 *  Nota: simplemente es una wrapper   *
 *        function para semctl         *
 ***************************************/
int create_binsem(int semnum)
{
    int i, semid;
    union semun{
        int val;
        struct semid_ds* buf;
        unsigned short* array;
    } semun;

    semun.val = 1;

    if ((semid = semget(IPC_PRIVATE, semnum, IPC_CREAT | 0777)) == -1)
        return -1;

    // Casi es más difícil hacerlo con SETALL
    for (i = 0; i < semnum; ++i){
        if (semctl(semid, i, SETVAL, semun) == -1){
            if (semctl(semid, 0, IPC_RMID) == -1){
                perror("semctl: IPC_RMID");
            }
            perror("semctl");
            return -1;
        }
    }

    return semid;
}

/***************************************
 *  semwait                            *
 ***************************************
 *                                     *
 *  Implementa la operacion P ('wait') *
 *  de un semaforo de id semid. La     *
 *  comprobacion se hace sobre el      *
 *  semnum-esimo semaforo del array    *
 *  identificado por semid.            *
 *                                     *
 *  Return: 0 si tiene exito, -1 si    *
 *          se produce algun error     *
 *                                     *
 *  Nota: simplemente es una wrapper   *
 *        function para semop          *
 ***************************************/
int semwait(int semid, int semnum)
{
    int rvalue;
    struct sembuf sops[1];

    sops[0].sem_num = semnum;
    sops[0].sem_op = -1;
    sops[0].sem_flg = 0;

    rvalue = semop(semid, sops, 1);

    if (rvalue == -1) {
        perror("semop");
    }

    return rvalue;
}

/***************************************
 *  semwait                            *
 ***************************************
 *                                     *
 *  Implementa la operacion P ('wait') *
 *  de un semaforo de id semid. La     *
 *  comprobacion se hace sobre el      *
 *  semnum-esimo semaforo del array    *
 *  identificado por semid.            *
 *                                     *
 *  Return: 0 si tiene exito, -1 si    *
 *          se produce algun error     *
 *                                     *
 *  Nota: simplemente es una wrapper   *
 *        function para semop          *
 ***************************************/
int semwait_val(int semid, int semnum, int val)
{
    int rvalue;
    struct sembuf sops[1];

    sops[0].sem_num = semnum;
    sops[0].sem_op = (-1)*val;
    sops[0].sem_flg = 0;

    rvalue = semop(semid, sops, 1);

    if (rvalue == -1) {
        perror("semop");
    }

    return rvalue;
}

/******************************************
 *  semwait                               *
 ******************************************
 *                                        *
 *  Espera hasta que el semaforo indicado *
 *  por los argumentos (que funcionan     *
 *  como en semwait) tenga valor 0.       *
 *                                        *
 *  Return: 0 si tiene exito, -1 si       *
 *          se produce algun error        *
 *                                        *
 *  Nota: simplemente es una wrapper      *
 *        function para semop             *
 ******************************************/
int semwait_0(int semid, int semnum)
{
    int rvalue;
    struct sembuf sops[1];

    sops[0].sem_num = semnum;
    sops[0].sem_op = 0;
    sops[0].sem_flg = 0;

    rvalue = semop(semid, sops, 1);

    if (rvalue == -1) {
        perror("semop");
	return -1;
    }

    return rvalue;
}

/******************************************
 *  semsig                                *
 ******************************************
 *                                        *
 *  Implementa la operacion V ('signal')  *
 *  de un semaforo de id semid. La        *
 *  operacion se hace sobre el semnum-    *
 *  esimo semaforo del array identificado *
 *  por semid.                            *
 *                                        *
 *  Return: 0 si tiene exito, -1 si       *
 *          se produce algun error        *
 *                                        *
 *  Nota: simplemente es una wrapper      *
 *        function para semop             *
 ******************************************/
int semsig(int semid, int semnum)
{
    int rvalue;
    struct sembuf sops[1];

    sops[0].sem_num = semnum;
    sops[0].sem_op = 1;
    sops[0].sem_flg = 0;

    rvalue = semop(semid, sops, 1);

    if (rvalue == -1) {
        perror("semop");
        return -1;
    } else {
        return rvalue;
    }
}

/******************************************
 *  semsig                                *
 ******************************************
 *                                        *
 *  Implementa la operacion V ('signal')  *
 *  de un semaforo de id semid. La        *
 *  operacion se hace sobre el semnum-    *
 *  esimo semaforo del array identificado *
 *  por semid.                            *
 *                                        *
 *  Return: 0 si tiene exito, -1 si       *
 *          se produce algun error        *
 *                                        *
 *  Nota: simplemente es una wrapper      *
 *        function para semop             *
 ******************************************/
int semsig_val(int semid, int semnum, int val)
{
    int rvalue;
    struct sembuf sops[1];

    sops[0].sem_num = semnum;
    sops[0].sem_op = val;
    sops[0].sem_flg = 0;

    rvalue = semop(semid, sops, 1);

    if (rvalue == -1) {
        perror("semop");
        return -1;
    } else {
        return rvalue;
    }
}


int get_semval(int semid, int semnum)
{
    int rvalue;

    if ((rvalue = semctl(semid, semnum, GETVAL)) == -1){
        perror("semctl");
    }

    return rvalue;
}

/******************************************
 *  set_semval                            *
 ******************************************
 *                                        *
 *  Función que asigna el valor val al    *
 *  semaforo. La                          *
 *  operacion se hace sobre el semnum-    *
 *  esimo semaforo del array identificado *
 *  por semid.                            *
 *                                        *
 *  Return: 0 si tiene exito, -1 si       *
 *          se produce algun error        *
 *                                        *
 *  Nota: simplemente es una wrapper      *
 *        function para semop             *
 ******************************************/
int set_semval(int semid, int semnum, int val)
{
    int rvalue;
    union semun{
        int val;
        struct semid_ds* buf;
        unsigned short* array;
    } semun;

    semun.val = val;

    if ((rvalue = semctl(semid, semnum, SETVAL, semun)) == -1){
        perror("semctl");
    }

    return rvalue;
}

int primer_ajuste(HCoche hc)
{
#ifdef PRIMER_AJUSTE_OFF //__DEBUG__
	return -2;
#else

    int c_length;
    int i, j;

    c_length = PARKING_getLongitud(hc);

    // Las llamadas a los ajustes son secuenciales, por lo que el semáforo
    // no 'molesta' como podría si hubiese varias llamadas a estas funciones
    // concurrentemente


    for (i = 0; i <= ROAD_LENGTH - c_length ; ++i){

        if (ipc.shmaddr->road[PRIMER_AJUSTE][LOTS][i] == EMPTY){

            j = i;
            while (j < i + c_length){
                if (EMPTY == ipc.shmaddr->road[PRIMER_AJUSTE][LOTS][j])
                    ++j;
                else
                    break;
            }

            if ((j == i + c_length)){

                for (j = i; j < i + c_length; j++){
                    ipc.shmaddr->road[PRIMER_AJUSTE][LOTS][j] = RESERVED;
		        }

                return i;
            }
        }
    }

    return -1;

#endif
}

int mejor_ajuste(HCoche hc)
{
#ifdef MEJOR_AJUSTE_OFF //__DEBUG__

	return -2;

#else

    int i, j;
    int c_length;
    int size, n_huecos, bestfit;
    int huecos[ROAD_LENGTH] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    };

    c_length = PARKING_getLongitud(hc);

    /* Buscamos los huecos en que el coche cabe */
    n_huecos = 0;
    i = 0;
    while (i <= ROAD_LENGTH - c_length){

        size = 0;

        if (EMPTY == ipc.shmaddr->road[MEJOR_AJUSTE][LOTS][i]){

            for (j = i; j < ROAD_LENGTH 
                && EMPTY == ipc.shmaddr->road[MEJOR_AJUSTE][LOTS][j]; ++j){
                ++size;
            }

            if (size >= c_length){
                huecos[i] = size;
                ++n_huecos;
                i += size;
            } else {
                ++i;
            }

        } else {
            ++i;
        }
    }

    if (n_huecos < 1){
        return -1;
    }

    /* Seleccionamos el hueco más pequeño */
    for (i = 0; i < ROAD_LENGTH; ++i){
        if (huecos[i] != 0){
            bestfit = i;
            break;
        }
    }

    for (i = bestfit + 1; i < ROAD_LENGTH; ++i){
        if (huecos[i] != 0 && huecos[i] < huecos[bestfit]){
            bestfit = i;
        }
    }

    for (i = bestfit; i < bestfit + c_length; ++i)
        ipc.shmaddr->road[MEJOR_AJUSTE][LOTS][i] = RESERVED;

    return bestfit;
    
#endif
}

int peor_ajuste(HCoche hc)
{
#ifdef PEOR_AJUSTE_OFF //__DEBUG__

	return -2;

#else

    int i, j;
    int c_length;
    int size, n_huecos, worstfit;
    int huecos[ROAD_LENGTH] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    };

    c_length = PARKING_getLongitud(hc);

    /* Buscamos los huecos en que el coche cabe */
    n_huecos = 0;
    i = 0;
    while (i <= ROAD_LENGTH - c_length){

        size = 0;

        if (EMPTY == ipc.shmaddr->road[PEOR_AJUSTE][LOTS][i]){

            for (j = i; j < ROAD_LENGTH 
                && EMPTY == ipc.shmaddr->road[PEOR_AJUSTE][LOTS][j]; ++j){
                ++size;
            }

            if (size >= c_length){
                huecos[i] = size;
                ++n_huecos;
                i += size;
            } else {
                ++i;
            }

        } else {
            ++i;
        }
    }

    if (n_huecos < 1){
        return -1;
    }

    /* Seleccionamos el hueco más grande */
    for (i = 0; i < ROAD_LENGTH; ++i){
        if (huecos[i] != 0){
            worstfit = i;
            break;
        }
    }

    for (i = worstfit + 1; i < ROAD_LENGTH; ++i){
        if (huecos[i] != 0 && huecos[i] > huecos[worstfit]){
            worstfit = i;
        }
    }

    for (i = worstfit; i < worstfit + c_length; ++i)
        ipc.shmaddr->road[PEOR_AJUSTE][LOTS][i] = RESERVED;

    return worstfit;

#endif
}

int siguiente_ajuste(HCoche hc)
{
#ifdef SIGUIENTE_AJUSTE_OFF //__DEBUG__

    return -2;

#else

    static int last_pos = -1;
    int c_length;
    int i, first_spot, free_spot;
    
    c_length = PARKING_getLongitud(hc);

    /* Se podría hacer con el módulo de la posición last_pos % ROAD_LENGTH,
     * pero creo que queda más complejo por el tratamiento especial al valor
     * de last_pos 0. Por ello, corrijo manualmente cuando last_pos se pasa
     * de ROAD_LENGTH -1
     */

        if (last_pos > -1){
            if (EMPTY != ipc.shmaddr->road[SIGUIENTE_AJUSTE][LOTS][last_pos]){
                first_spot = siguiente_ajuste_primer_hueco(last_pos);
                if (-1 == first_spot){
                    return -1;
                }
            } else {
                first_spot = last_pos;
		while(ipc.shmaddr->road[SIGUIENTE_AJUSTE][LOTS][first_spot] == EMPTY && first_spot>0) first_spot--;
            }
        } else {
            first_spot = 0;
        }

        free_spot = siguiente_ajuste_ajusta_primer(c_length, 
                                                   first_spot, 
                                                   ROAD_LENGTH);

        if (-1 == free_spot){
            free_spot = siguiente_ajuste_ajusta_primer(c_length, 0, 
                                                       first_spot);
        }

        if (free_spot != -1){

            for (i = free_spot; i < free_spot + c_length; ++i){
                ipc.shmaddr->road[SIGUIENTE_AJUSTE][LOTS][i] = RESERVED;
            }

            last_pos = free_spot;
        }

    return free_spot;

#endif

}

int siguiente_ajuste_primer_hueco(int last_pos)
{
    int i;

    for (i = last_pos + 1; i < ROAD_LENGTH; ++i){
        if (EMPTY == ipc.shmaddr->road[SIGUIENTE_AJUSTE][LOTS][i])
            return i;
    }

    for (i = 0; i < last_pos; ++i){
        if (EMPTY == ipc.shmaddr->road[SIGUIENTE_AJUSTE][LOTS][i])
            return i;
    }

    return -1;
}

int siguiente_ajuste_ajusta_primer(int c_length, int a, int b)
{
    int i, j;

    for (i = a; i <= b - c_length ; ++i){

        if (ipc.shmaddr->road[SIGUIENTE_AJUSTE][LOTS][i] == EMPTY){

            for (j = i; j < i + c_length; j++){
                if (ipc.shmaddr->road[SIGUIENTE_AJUSTE][LOTS][j] != EMPTY)
                    break;
            }

            if (j == i + c_length){
                return i;
            }
        }
    }

    return -1;
}

void aparcar_commit(HCoche hc)
{
    switch(PARKING_getAlgoritmo(hc)){

    case PRIMER_AJUSTE:
        if (set_semval(ipc.semid, 
        ipc.sem_0 + SEM_ORDEN_COCHES(PRIMER_AJUSTE),
        PARKING_getNUmero(hc) + 1) == -1){
            perror("aparcar_commit: set_semval");
            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                perror("aparcar_commit: kill");
            }
            return ;
        }
    break;
    case SIGUIENTE_AJUSTE:
        if (set_semval(ipc.semid, 
        ipc.sem_0 + SEM_ORDEN_COCHES(SIGUIENTE_AJUSTE),
        PARKING_getNUmero(hc) + 1)  == -1){
            perror("aparcar_commit: set_semval");
            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                perror("aparcar_commit: kill");
            }
            return ;
        }
    break;
    case MEJOR_AJUSTE:
        if (set_semval(ipc.semid, 
        ipc.sem_0 + SEM_ORDEN_COCHES(MEJOR_AJUSTE),
        PARKING_getNUmero(hc) + 1)  == -1){
            perror("aparcar_commit: set_semval");
            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                perror("aparcar_commit: kill");
            }
            return ;
        }
    break;
    case PEOR_AJUSTE:
        if (set_semval(ipc.semid, 
        ipc.sem_0 + SEM_ORDEN_COCHES(PEOR_AJUSTE),
        PARKING_getNUmero(hc) + 1)  == -1){
            perror("aparcar_commit: set_semval");
            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                perror("aparcar_commit: kill");
            }
            return ;
        }
    break;
    }
}

void permiso_avance(HCoche hc)
{
	struct msg_permiso_avance msg;
	int i;
	int X_inicio, X_fin, Y_inicio, Y_fin, algoritmo;

	X_inicio = PARKING_getX(hc);
	X_fin = PARKING_getX2(hc);
	Y_inicio = PARKING_getY(hc);
	Y_fin = PARKING_getY2(hc);
	algoritmo = PARKING_getAlgoritmo(hc);

	if (X_fin >= 0
    && X_fin < ROAD_LENGTH
	&& Y_inicio == Y_fin){ // Movimiento horizontal

		if (msgrcv(ipc.msgid, &msg, sizeof(msg) - sizeof(long), 
		    TIPO_MSG_PERMISO_AVANCE(algoritmo, X_fin), 0) == -1){

		    perror("permiso_avance: msgrcv");
            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                perror("permiso_avance: kill");
            }

		}

	} else if (Y_fin == LANE && Y_inicio == STRIPE){ // Desaparcando

		for (i = X_fin + PARKING_getLongitud(hc) - 1; i >= X_fin; --i){
			if (msgrcv(ipc.msgid, &msg, sizeof(msg) - sizeof(long),
                TIPO_MSG_PERMISO_AVANCE(algoritmo, i), 0) == -1){
                perror("permiso_avance: msgrcv");
                if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                    perror("permiso_avance: kill");
                }
                break;
            }
		}

	}
}

void permiso_avance_commit(HCoche hc)
{
    struct msg_permiso_avance msg;
	int i;
	int X_inicio, X_fin, Y_inicio, Y_fin, algoritmo, c_length;

	X_inicio = PARKING_getX2(hc);
	X_fin = PARKING_getX(hc);
	Y_inicio = PARKING_getY2(hc);
	Y_fin = PARKING_getY(hc);
	algoritmo = PARKING_getAlgoritmo(hc);
	c_length = PARKING_getLongitud(hc);

    if (X_fin + c_length >= 0
    && X_fin + c_length - 1 < ROAD_LENGTH
    && Y_inicio == Y_fin){ // Movimiento horizontal

        msg.alg_pos_id = TIPO_MSG_PERMISO_AVANCE(algoritmo, 
                                                 X_inicio + c_length - 1);
        if (msgsnd(ipc.msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1){
            perror("permiso_avance_commit: msgsnd");
            if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                perror("permiso_avance_commit: kill");
            }
        }

    } else if (Y_fin == LANE && Y_inicio == STRIPE){ // Desaparcando

        for (i = X_fin + c_length - 1; i >= X_fin; --i){
            ipc.shmaddr->road[algoritmo][LOTS][i] = EMPTY;
        }

    } else if (Y_inicio == LANE && Y_fin == STRIPE){ // Aparcando

        for (i = X_inicio + c_length - 1; i >= X_inicio; --i){
            msg.alg_pos_id = TIPO_MSG_PERMISO_AVANCE(algoritmo, i);
            if (msgsnd(ipc.msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1){
                perror("permiso_avance_commit: msgsnd");
                if (kill(ipc.shmaddr->pid[TIMER_ADDR], SIGUSR1) == -1){
                    perror("permiso_avance_commit: kill");
                }
                break;
            }
        }

    }
}

/******************************************
 *  calculate_shmaddr                     *
 ******************************************
 *                                        *
 *  Función que devuelve el puntero al    *
 *  espacio de la zona de memoria donde   *
 *  se almacenara la biblioteca           *
 *                                        *
 *  Return: puntero a la zona de memoria  *
 *	    si tiene exito, NULL si       *
 *          se produce algun error        *
 *                                        *
 ******************************************/
void * calculate_shmaddr(struct ipc_rsrc * ipc)
{
    if (ipc == NULL || ipc->lib_shmaddr == NULL)
        return NULL;
    else
        // Posiciones que ocupa la biblioteca en memoria
        return (void *) ipc->lib_shmaddr + ipc->lib_shmsize/sizeof(char); 
}

/******************************************
 *  calculate_pid_addr                    *
 ******************************************
 *                                        *
 *  Función que devuelve el puntero al    *
 *  espacio de la zona de memoria donde   *
 *  se almacenara un struct de tipo       *
 *  ipc_rsrc                              *
 *                                        *
 *  Return: puntero a la zona de memoria  *
 *	    si tiene exito, NULL si       *
 *          se produce algun error        *
 *                                        *
 ******************************************/
void * calculate_pid_addr(struct ipc_rsrc * ipc)
{
    if (ipc == NULL || ipc->shmaddr == NULL)
        return NULL;
    else
        return (void *) (ipc->shmaddr + 1); 
}

/******************************************
 *  init_shm                              *
 ******************************************
 *                                        *
 *  Función que inicializa el espacio de  *
 *  la zona de memoria compartida a -1    *
 *                                        *
 *  Return: 0 si tiene exito, -1 si       *
 *          se produce algun error        *
 *                                        *
 ******************************************/
int init_shm(struct shm_rsrc * shmaddr)
{
    int i, j;
    if (shmaddr == NULL)
        return -1;

    shmaddr->pid = NULL;
    shmaddr->n_children = 0;

    for (j = 0; j < ROAD_LENGTH; ++j){
        shmaddr->road[PRIMER_AJUSTE][LOTS][j] = EMPTY;
        shmaddr->road[SIGUIENTE_AJUSTE][LOTS][j] = EMPTY;
        shmaddr->road[MEJOR_AJUSTE][LOTS][j] = EMPTY;
        shmaddr->road[PEOR_AJUSTE][LOTS][j] = EMPTY;
    }

    for (j = 0; j < ROAD_LENGTH; ++j){
        shmaddr->road[PRIMER_AJUSTE][STRIPE][j] = EMPTY;
        shmaddr->road[SIGUIENTE_AJUSTE][STRIPE][j] = EMPTY;
        shmaddr->road[MEJOR_AJUSTE][STRIPE][j] = EMPTY;
        shmaddr->road[PEOR_AJUSTE][STRIPE][j] = EMPTY;
    }

    for (j = 0; j < ROAD_LENGTH; ++j){
        shmaddr->road[PRIMER_AJUSTE][LANE][j] = EMPTY;
        shmaddr->road[SIGUIENTE_AJUSTE][LANE][j] = EMPTY;
        shmaddr->road[MEJOR_AJUSTE][LANE][j] = EMPTY;
        shmaddr->road[PEOR_AJUSTE][LANE][j] = EMPTY;
    }

    shmaddr->mensajes[N_MSG_APARCAR] = 0;
    shmaddr->mensajes[N_MSG_DESAPARCAR] = 0;

    return 0;
}

/******************************************
 *  init_shm                              *
 ******************************************
 *                                        *
 *  Función que inicializa el espacio de  *
 *  la zona de memoria compartida a -1    *
 *                                        *
 *  Return: 0 si tiene exito, -1 si       *
 *          se produce algun error        *
 *                                        *
 ******************************************/
int init_ipc(struct ipc_rsrc * ipc, int n_chofer)
{
	int i, j;
	struct msg_permiso_avance msg;

    if (ipc == NULL)
        return -1;

    /* Obtenemos el numero de semaforos y de memoria compartida
     * necesaria para la simulacion
     * A esto habrá que sumarle la que necesite este programa
     * sem_0 indica el indice del primero de nuestros semaforos
     * o mejor dicho, el nº de semaforos que utiliza la biblioteca
     */
    ipc->sem_0 = PARKING_getNSemAforos();
    ipc->nsems = ipc->sem_0 + N_ADD_SEMS;
    ipc->lib_shmsize = PARKING_getTamaNoMemoriaCompartida();

    /* El numero de PID a guardar es:
     *     - El del padre
     *     - El del gestor
     *     - El del cronometro
     *     - El de los choferes
     * Tambien hay que guardar la estructura de tipo shm_rsrc
     */
    ipc->shmsize = sizeof(struct shm_rsrc) 
                + (N_PIDS + n_chofer)*sizeof(pid_t);
    
    /* Reservamos los recursos de IPC que se requieren */

    // SEMAFOROS
    if ((ipc->semid = create_binsem(ipc->nsems)) == -1){
        perror("create_binsem");
        return -1;
    }


    // COLA DE MENSAJES
    if ((ipc->msgid = create_msg()) == -1){
        perror("create_msh: msgget");
        if (semctl(ipc->semid, 0, IPC_RMID) == -1){
            perror("semctl");
        }
        return -1;
    }

	for (j = 0; j < ROAD_LENGTH; ++j){
		msg.alg_pos_id = TIPO_MSG_PERMISO_AVANCE(PRIMER_AJUSTE, j);
		if (msgsnd(ipc->msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1){
            perror("msgsnd");
            if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
                perror("msgctl");
            }
            if (semctl(ipc->semid, 0, IPC_RMID) == -1){
                perror("semctl");
            }
            return -1;
        }
	}

    for (j = 0; j < ROAD_LENGTH; ++j){
        msg.alg_pos_id = TIPO_MSG_PERMISO_AVANCE(SIGUIENTE_AJUSTE, j);
        if (msgsnd(ipc->msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1){
            perror("msgsnd");
            if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
                perror("msgctl");
            }
            if (semctl(ipc->semid, 0, IPC_RMID) == -1){
                perror("semctl");
            }
            return -1;
        }
    }

    for (j = 0; j < ROAD_LENGTH; ++j){
        msg.alg_pos_id = TIPO_MSG_PERMISO_AVANCE(MEJOR_AJUSTE, j);
        if (msgsnd(ipc->msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1){
            perror("msgsnd");
            if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
                perror("msgctl");
            }
            if (semctl(ipc->semid, 0, IPC_RMID) == -1){
                perror("semctl");
            }
            return -1;
        }
    }

    for (j = 0; j < ROAD_LENGTH; ++j){
        msg.alg_pos_id = TIPO_MSG_PERMISO_AVANCE(PEOR_AJUSTE, j);
        if (msgsnd(ipc->msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1){
            perror("msgsnd");
            if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
                perror("msgctl");
            }
            if (semctl(ipc->semid, 0, IPC_RMID) == -1){
                perror("semctl");
            }
            return -1;
        }
    }


    // MEMORIA COMPARTIDA
    if  ((ipc->shmid = create_shmem(ipc->lib_shmsize + ipc->shmsize)) == -1){
        perror("create_shmem: shmget");
        if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
            perror("msgctl");
        }
        if (semctl(ipc->semid, 0, IPC_RMID) == -1){
            perror("semctl");
        }
        return -1;
    }

    if ((ipc->lib_shmaddr = shmat(ipc->shmid, NULL, 0)) == (char *) -1){
        perror("shmat");
        if (shmctl(ipc->shmid, IPC_RMID, 0) == -1){
            perror("shmctl");
        }
        if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
            perror("msgctl");
        }
        if (semctl(ipc->semid, 0, IPC_RMID) == -1){
            perror("semctl");
        }
        return -1;
    }


    /* Por simplicidad de trabajo, almacenamos la posicion de memoria 
     * compartida que usa este codigo (y no el de la simulacion)
     * El tamaño que ocuparan los procesos, la carretera y la variable compartida
     * entre el chofer y el gestor
     */
 
    if((ipc->shmaddr = calculate_shmaddr(ipc)) == NULL){
	    perror("calculate_shmaddr");
        if (shmctl(ipc->shmid, IPC_RMID, 0) == -1){
            perror("shmctl");
        }
        if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
            perror("msgctl");
        }
        if (semctl(ipc->semid, 0, IPC_RMID) == -1){
            perror("semctl");
        }
        return -1;
    }

    // Inicializamos la memoria compartida a valores por defecto
    if (init_shm(ipc->shmaddr) == -1){
        perror("init_shm");
        if (shmctl(ipc->shmid, IPC_RMID, 0) == -1){
            perror("shmctl");
        }
        if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
            perror("msgctl");
        }
        if (semctl(ipc->semid, 0, IPC_RMID) == -1){
            perror("semctl");
        }
        return -1;
    }

    // Almacenamos el resto de la informacion necesaria
    if((ipc->shmaddr->pid = calculate_pid_addr(ipc)) == NULL){
	    perror("calculate_pid_addr");
        if (shmctl(ipc->shmid, IPC_RMID, 0) == -1){
            perror("shmctl");
        }
        if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
            perror("msgctl");
        }
        if (semctl(ipc->semid, 0, IPC_RMID) == -1){
            perror("semctl");
        }
        return -1;
    }

    ipc->shmaddr->n_children = N_PIDS + n_chofer - 1;

    /* Inicializamos el semaforo que controla el acceso de los choferes
     * a los mensajes y el semaforo que utilizara permiso avance,ambos a 0
     */

    if((set_semval(ipc->semid, ipc->sem_0 + SEM_MSG, 0)) == -1){
    	perror("set_semval:SEM_MSG");
        if (shmctl(ipc->shmid, IPC_RMID, 0) == -1){
            perror("shmctl");
        }
        if (msgctl(ipc->msgid, IPC_RMID, NULL) == -1){
            perror("msgctl");
        }
        if (semctl(ipc->semid, 0, IPC_RMID) == -1){
            perror("semctl");
        }
    	return -1;
    }

    return 0;
}
