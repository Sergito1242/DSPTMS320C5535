#include "stdio.h"
#include "ezdsp5535.h"
#include <csl_mmcsd.h>
#include "ezdsp5535_sar.h"
#include <float.h>
#include <math.h>

#define MAX_TEMPLATE_SIZE 78   /* Tamaño máximo de plantilla (ventanas) */
#define MAX_MFCC_COEFS 13      /* Número máximo de coeficientes MFCC por ventana */
#define MAX_TEMPLATES 20       /* Número máximo de plantillas (ejemplos) */
#define BUFFER_MAX_SIZE    (1024u)
#define MAX_MFCC_COEFS 13

typedef struct {
    Uint32 sector_location;   // Sector donde están guardados los MFCCs en SD
    int label;                // Etiqueta del comando
    char name[32];            // Nombre descriptivo
} Template;

static Template templates[MAX_TEMPLATES];
static int num_templates = 0;
extern CSL_MmcsdHandle  mmcsdHandle;
Uint16 RReadBuff[BUFFER_MAX_SIZE/2];
double tempBuff[BUFFER_MAX_SIZE];

double Mfcc_voice[MAX_TEMPLATE_SIZE][MAX_MFCC_COEFS];
double Mfcc_template[MAX_TEMPLATE_SIZE][MAX_MFCC_COEFS];
Uint16 command;

extern Int16 Configuracion( );
extern void Close();
extern Int16 aic3204_listen( );
extern Int16 recognize_voice_command( Uint32 audio_sector);
extern void init_templates();

double uint16ToDouble(Uint16 value);
Uint16 extract_mfcc_from_sd(Uint32 sector_start, double output_buffer[MAX_TEMPLATE_SIZE][MAX_MFCC_COEFS]);
Int16 classify_command();
void init_templates(void);
double uint16_array_to_double(const Uint16 input[2]);
void LecturaMemoria(Uint32 sector_start);

extern double Voice_Mfcc[78][13];
extern Uint32 Num_Windows;
extern double Templest[5][78][13];
extern double result;

////////////////////////////////////////////////////////////////////////////
//extern dtw_custom(Uint16 len1, Uint16 len2, Uint16 num_coefs, Uint16 i);
extern void inicializarGMM();
extern void inicializar2();
extern double Probabilidad_GMM(Uint16 Num_Comando, Uint16 num_ventanas);
////////////////////////////////////////////////////////////////////////////

void main( void )
{
    /* Initialize BSL */
    EZDSP5535_init( );
    if(EZDSP5535_SAR_init())
    {
        printf("Error al inicializar el módulo SAR\n");
        return;
    }
    Configuracion();
    init_templates();
    inicializarGMM();
    printf("Presione el SW1 para escuchar comando\n");
    while(1)
    {
        if(EZDSP5535_SAR_getKey() == SW1)
        {
            aic3204_listen( );
            command = classify_command();
            switch(command)
            {
            case 0:
                printf("Luz");
                break;
            case 1:
                printf("Calefaccion");
                break;
            case 2:
                printf("Puerta");
                break;
            case 3:
                printf("Alarma");
                break;
            case 4:
                printf("Ventilador");
                break;
            }
            printf("\nPresione el SW1 para reconocer el comando\n");
        }
    }
}

Int16 classify_command()
{
    double similitud;
    int best_match = -1;
    Uint32 i,index,h,t;

    Uint16 WindowsVoice = Num_Windows;

//    similitud = DBL_MAX;// DTW
    similitud = -DBL_MAX;//GMM

    for (i = 0; i < num_templates; i++)
    {

///////////////dtw

//        dtw_custom(WindowsVoice, templates[i].sector_location, 13, i);
//
//        /* Si la distancia es menor que un umbral, considerar como coincidencia */
//        if (result < similitud) {
//            similitud = result;
//            best_match = templates[i].label;
//        }

///////////////GMM

        Probabilidad_GMM( i, WindowsVoice);

        if (result > similitud) {
            similitud = result;
            best_match = templates[i].label;
        }

    }

    /* Si la mejor distancia es muy alta, podría no ser ningún comando conocido */
//    if (similitud > 10000.0) { /* Ajustar este umbral según tus datos */
//        return -1;
//    }
    return best_match;
}

double uint16_array_to_double(const Uint16 input[2]) {
    union {
        double d;
        Uint16 arr[4];
    } u;
    u.arr[0] = input[0];
    u.arr[1] = input[1];
    u.arr[2] = 0;
    u.arr[3] = 0;
    return u.d;
}

void LecturaMemoria(Uint32 sector_start)
{
   Uint16 array[2],index,i;
   Uint32 block = sector_start*512;

   MMC_read(mmcsdHandle, block, 1024, RReadBuff);

   index = 0;

   for (i = 0; i < 1024; i++) {
       array[0] = RReadBuff[(i-index)*2];
       array[1] = RReadBuff[((i-index)*2)+1];
       tempBuff[i] = uint16_array_to_double(array);
       printf("%f ", tempBuff[i]);
       if((i % 256) == 0 && i > 0) {
           sector_start++;
           block = sector_start*512;
           MMC_read(mmcsdHandle, block, 1024, RReadBuff);
           index += 256;
       }
   }
   printf("Lectura Completada");
}

Uint16 extract_mfcc_from_sd(Uint32 sector_start, double output_buffer[MAX_TEMPLATE_SIZE][MAX_MFCC_COEFS]) {
    Uint16 numValidWindows = 0;
    Uint32 i, j, pos;

    LecturaMemoria(sector_start);

    int maxWindows = BUFFER_MAX_SIZE / MAX_MFCC_COEFS;
    if (maxWindows > MAX_TEMPLATE_SIZE) maxWindows = MAX_TEMPLATE_SIZE;

    pos = 0;
    for (i = 0; i < maxWindows; i++)
    {
        int nonZeroValues = 0;
        for (j = 0; j < MAX_MFCC_COEFS && pos + j < BUFFER_MAX_SIZE; j++) {
            if (tempBuff[pos + j] != 0.0) nonZeroValues++;
        }
        if (nonZeroValues == 0) break;

        for (j = 0; j < MAX_MFCC_COEFS && pos + j < BUFFER_MAX_SIZE; j++) {
            output_buffer[i][j] = tempBuff[pos + j];
        }

        numValidWindows++;
        pos += MAX_MFCC_COEFS;
    }

    return numValidWindows;
}

void init_templates() {
    // Plantilla 1 - Sector 300 y 301
    Template plantilla1 = {27, 0, "Luz"};


    // Plantilla 2 - Sector 302 y 303
    Template plantilla2 = {35, 1, "Calefaccion"};

    Template plantilla3 = {49, 2, "Puerta"};

    Template plantilla4 = {39, 3, "Alarma"};

    Template plantilla5 = {39, 4, "Televisor"};


    templates[num_templates++] = plantilla1;
    templates[num_templates++] = plantilla2;
    templates[num_templates++] = plantilla3;
    templates[num_templates++] = plantilla4;
    templates[num_templates++] = plantilla5;
}

double uint16ToDouble(Uint16 value) {
    /* Implementación específica según el formato de datos */
    double result;

    union {
        Uint16 uint16Value;
        double doubleValue;
    } converter;

    converter.uint16Value = value;
    result = converter.doubleValue;

    return result;
}

