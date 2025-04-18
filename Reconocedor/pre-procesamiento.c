#include "stdio.h"
#include "ezdsp5535.h"
#include "ezdsp5535_i2s.h"
#include "csl_i2s.h"
#include <csl_mmcsd.h>
#include <csl_intc.h>
#include <csl_general.h>
#include <soc.h>
#include "csl_gpt.h"
#include <math.h>

#define Time 2

#define N 512

#define M_PI 3.14159265358979323846


// Par�metros configurables

#define FRAME_SIZE 256

#define ENERGY_THRESHOLD 0.01

#define HP_COEFF 0.98

#define Solapamiento 256



// SD card Buffer size in Bytes

#define BUFFER_MAX_SIZE    (1024u)

// Coeficientes del filtro

const double cn[] = {0.9870 , -2.9610 ,   2.9610,   -0.9870};

const double cd[] = {1.0000,   -2.9738  ,  2.9480 ,  -0.9742};

#define FILTER_ORDER 3

// Buffers para las se�ales

float input_buffer[11] = {0,0,0,0,0,0,0,0,0,0,0};

float output_buffer[11] = {0,0,0,0,0,0,0,0,0,0,0};

float input_signal = 0;

float output_signal = 0;


Int16 sec, msec;
Uint16 RpReadBuff[BUFFER_MAX_SIZE/2];
Uint16 RpWritedBuff[BUFFER_MAX_SIZE/2];
Uint16 LpReadBuff[BUFFER_MAX_SIZE/2];
Uint16 LpWritedBuff[BUFFER_MAX_SIZE/2];



double Muestras[BUFFER_MAX_SIZE/2];
double Hop[Solapamiento];
double Ventana[BUFFER_MAX_SIZE/2];
double PreProcesSamples[1024];
double Voice_Mfcc[78][13];
Uint32 Num_Windows;

Uint16 preprocess_sample(Uint16 Addr);
double apply_filter(double input);
Int16 aic3204_listen( );
Int16 CSL_gptIntr(void);
interrupt void gpt0Isr(void);
extern void VECSTART(void);
void pre_emphasis();

///////FFT

void fft(double input[N]);
void Suma_comp(Uint16 salto);
void Mult_w(Uint16 salto);
void Suma_Real();
void fill_w();
Uint16 inverso(Uint16 val);

float X[N];
float W_R[N], W_I[N];
float Y_R[N], Y_I[N];
float output_real[N];
float output_imag[N];
double power_spectrum[N/2];

///////MFCC



#define N_MEL_FILTERS 20      // N�mero de filtros Mel
#define N_CEPS 13             // Coeficientes MFCC a extraer

void compute_mfcc();
void init_mel_filter_bank(float Fs);
void apply_mel_filters();
void compute_dct(const float log_energy[N_MEL_FILTERS]);
void generar_hamming(double hammingWindow[512]);
void Save_plantilla();



float mel_filters[N_MEL_FILTERS][N/2]; // Banco de filtros Mel (precalculado)
float mfcc[N_CEPS];           // Coeficientes MFCC finales
float log_energy[N_MEL_FILTERS];

///////



#define SILENCE_THRESHOLD   145
#define WINDOW_SIZE         128
#define MIN_SILENCE_WINDOWS 4

//prueba

Int16 prueba[512]={0};
Int16 prueba2[512]={0};
Uint16 flag=0;
extern CSL_MmcsdHandle  mmcsdHandle;
CSL_Handle    hGpt;
Int16 Hit = 0;
Int16  data1, data2;
Int16 ciclos=0;
Int16 ciclos2=0;

Int16 aic3204_listen( )
{

    Int16 sec, msec, sample;
    Uint16 i = 0;
    Uint32 Addr = 0;

    CSL_gptIntr();

    /* Play Loop for 5 seconds */

    for ( sec = 0 ; sec < Time ; sec++ )
    {
        for ( msec = 0 ; msec < 1000 ; msec++ )
        {
            for ( sample = 0 ; sample < 16; sample++ )
            {
                while(Hit!=1)
                {

                }

                Hit=0;

                /* Write 16-bit left channel Data */

                EZDSP5535_I2S_writeLeft(data1);

                RpWritedBuff[i]=(Uint16)(((Int32)data1)+32768);
                i++;

                if(i>=512)
                {
                    MMC_write(mmcsdHandle, Addr*512, BUFFER_MAX_SIZE, RpWritedBuff);

                    Addr++;
                    i=0;
                }
            }
        }
    }

    Addr=preprocess_sample(Addr);

    Uint32 k;
    for(k=0;k<Addr;k++)
    {
        MMC_read(mmcsdHandle, (k+200)*512, BUFFER_MAX_SIZE, RpReadBuff);

        for(i=0;i<512;i++)
        {
            Int16 processed_sample = (Int16)(((Int32)RpReadBuff[i])-32768);
            // Escribir muestra procesada al DAC
            while(Hit!=1)
            {

            }
            Hit=0;

            EZDSP5535_I2S_writeLeft(processed_sample);

        }

    }

    IRQ_globalDisable();
    IRQ_clearAll();
    IRQ_disableAll();
    GPT_stop(hGpt);
    GPT_reset(hGpt);
    GPT_close(hGpt);

    return 0;

}

Uint16 por_saber;

Uint16 preprocess_sample(Uint16 Addr)
{

    double hammingWindow[512];
    Uint32 u = 0, l = 0;
    Int32 energy = 0;
    Uint16 ventana = 128;
    Uint16 inicio = 7;
    Uint16 umbral = 170;
    Uint16 silent_count = 0;
    Uint16 i,h,t;
    Uint16 Coeficientes_Index = 0;
    Uint16 Flag = 1;

    generar_hamming(hammingWindow);

    init_mel_filter_bank(16000);

    for(u = 0; u < 1024; u++)
        PreProcesSamples[u] = 0.0;

    for(h=0;h<78;h++)
    {
        for(t=0;t<N_CEPS;t++)
        {
            Voice_Mfcc[h][t]= 0.0;
        }
    }

    Uint16 N_VENTANAS = 0;

    for(u = inicio; u < Addr ; u++)

    {

        MMC_read(mmcsdHandle, u * 512, BUFFER_MAX_SIZE, RpReadBuff);

        // Calcular energ�a

        energy = 0;

        for(l = 0; l < ventana; l++)
            energy += abs((Int16)(((Int32)RpReadBuff[l]) - 32768));

        energy /= ventana;

        if (energy < umbral) {

            silent_count++;

            if (silent_count >= 3)

                continue;
        } else {
            silent_count = 0;
        }

        double suma=0.0, suma_cuadrados=0.0;

        for(l = 0; l < 512; l++)
        {
            double valor =  (double)((Int16)(((Int32)RpReadBuff[l]) - 32768));
            Muestras[l] = valor;
            suma += valor;
            suma_cuadrados += valor * valor;
        }

        double media = suma / 512;

        double varianza = (suma_cuadrados / 512) - (media * media);

        double desviacion_estandar = sqrt(varianza);

        double rms = sqrt(suma_cuadrados / 512);

        for(l = 0; l < 512; l++)
        {
            //Muestras[l] = valor/rms;//normalizacion rms
            Muestras[l] = (Muestras[l] - media) / desviacion_estandar; //normalizacion z-score
        }

        pre_emphasis();

        if(Flag == 1)
        {
            for(i = 0; i < 512; i++)     Ventana[i] = Muestras[i] * hammingWindow[i];

//            // Imprimir Ventana
//
//            for(i = 0; i < 512; i++)
//
//                printf("%f,", Ventana[i]);
//
//            printf("\n");

            fft(Ventana);
            compute_mfcc();

            for(i = 0; i < N_CEPS; i++)
            {
                Voice_Mfcc[N_VENTANAS][i] = mfcc[i];
                PreProcesSamples[Coeficientes_Index++] = mfcc[i];
            }

            N_VENTANAS++;

            // Guardar mitad final para siguiente solapamiento

            for(i = Solapamiento; i < 512; i++) Hop[i - Solapamiento] = Muestras[i];

            Flag = 0;

        }

        else

        {

            // Ventana solapada: Hop + nueva mitad

            for(i = 0; i < 512; i++)

            {
                if(i < Solapamiento)
                    Ventana[i] = Hop[i] * hammingWindow[i];
                else
                    Ventana[i] = Muestras[i - Solapamiento] * hammingWindow[i];
            }

//                //Imprimir Ventana

//                for(i = 0; i < 512; i++)

//                    printf("%f,", Ventana[i]);

//                printf("\n");

            fft(Ventana);
            compute_mfcc();

            for(i = 0; i < N_CEPS; i++) {
                Voice_Mfcc[N_VENTANAS][i] = mfcc[i];
                PreProcesSamples[Coeficientes_Index++] = mfcc[i];
            }

            N_VENTANAS++;

            //Ventana completa actual

            for(i = 0; i < 512; i++)     Ventana[i] = Muestras[i] * hammingWindow[i];

//                //Imprimir Ventana

//                for(i = 0; i < 512; i++)

//                    printf("%f,", Ventana[i]);

//                printf("\n");

            fft(Ventana);
            compute_mfcc();

            for(i = 0; i < N_CEPS; i++)
            {
                Voice_Mfcc[N_VENTANAS][i] = mfcc[i];
                PreProcesSamples[Coeficientes_Index++] = mfcc[i];
            }

            N_VENTANAS++;

            // Guardar mitad final para siguiente bloque

            for(i = Solapamiento; i < 512; i++)  Hop[i - Solapamiento] = Muestras[i];

        }

    }

    Num_Windows = N_VENTANAS;
    //Save_plantilla();

//    for(h=0;h<78;h++)
//    {
//        printf(",");
//        printf("{");
//        for(t=0;t<N_CEPS;t++)
//        {
//            if(t<12)
//                printf("%f, ",Voice_Mfcc[h][t]);
//            else
//                printf("%f ",Voice_Mfcc[h][t]);
//        }
//        printf("}");
//    }

//     copiar python
    printf("[\n");
    for (h = 0; h < 78; h++) {
        printf("  [");
        for (t = 0; t < N_CEPS; t++) {
            if (t < N_CEPS - 1)
                printf("%f, ", Voice_Mfcc[h][t]);
            else
                printf("%f", Voice_Mfcc[h][t]);
        }
        if (h < 78 - 1)
            printf("],\n");
        else
            printf("]\n");
    }
    printf("]\n");

    return 0;

}

void double_to_uint16_array(double input, Uint16 output[2]) {
    union {
        double d;
        Uint16 arr[2];
    } u;
    u.d = input;
    output[0] = u.arr[0];
    output[1] = u.arr[1];
}

double uint16_array_to_double2(const Uint16 input[2]) {
    union {
        double d;
        Uint16 arr[2];
    } u;
    u.arr[0] = input[0];
    u.arr[1] = input[1];
    return u.d;
}

void Save_plantilla()
{
    Uint32 index, j;
    Uint32 sector = 174;
    Uint16 array[2];
    Uint32 buff_index = 0;

    for(index = 0; index < 1024; index++)
    {
        printf("%f ",PreProcesSamples[index]);
        double_to_uint16_array(PreProcesSamples[index], array);
        for(j = 0; j < 2; j++)
        {
            RpWritedBuff[buff_index++] = array[j];
        }
        if(buff_index >= 512)
        {
            MMC_write(mmcsdHandle, sector * 512, BUFFER_MAX_SIZE, RpWritedBuff);
            sector++;
            buff_index = 0;
        }
    }
    printf("Comando Guardado");
}



void generar_hamming(double hammingWindow[512])

{

    Uint16 i;

    for (i = 0; i < 512; i++)

    {

        hammingWindow[i] = 0.54 - 0.46 * cos((2 * M_PI * i) / (512 - 1));

    }

}



void pre_emphasis() {

    double Alpha = 0.97;

    double anterior = Muestras[0];

    double actual;

    int i;

    for (i = 1; i < 512; i++) {

        actual = Muestras[i];

        Muestras[i] = Muestras[i] - Alpha * anterior;

        anterior = actual;

    }

}





double apply_filter(double input) {

    // Shift buffers

    int i;

    for (i = FILTER_ORDER; i > 0; i--) {

        input_buffer[i] = input_buffer[i - 1];

        output_buffer[i] = output_buffer[i - 1];

    }



    // Agregar nueva entrada

    input_buffer[0] = input;



    // Calcular salida del filtro

    double output = 0.0;

    for (i = 0; i <= FILTER_ORDER; i++) {

        output += cn[i] * input_buffer[i];

        if (i > 0) {

            output -= cd[i] * output_buffer[i];

        }

    }



    // Guardar salida actual

    output_buffer[0] = output;



    return output;

}



////////////////////////////////////////FFT



void fft(double input[N]) {

    Uint16 i,salto;

    // Aplicar bit-reverso al arreglo de entrada

    for ( i = 0; i < N; i++) {

        X[inverso(i)] = input[i];

    }



    // Calcular factores twiddle

    fill_w();



    // Primera etapa de la FFT

    Suma_Real();



    // Etapas posteriores de la FFT

    for ( salto = 4; salto <= N; salto *= 2) {

        Mult_w(salto);

        Suma_comp(salto);

    }



    // Normalizar los resultados

    for ( i = 0; i < N; i++) {

        output_real[i] = Y_R[i] / N;

        output_imag[i] = Y_I[i] / N;

    }

    for ( i = 0; i < N/2; i++) {

        power_spectrum[i]=(output_real[i]*output_real[i]) + (output_imag[i]*output_imag[i]);
//        printf("%f ", power_spectrum[i]);
    }

}



Uint16 inverso(Uint16 val) {

    Uint16 inv = 0;

    Uint16 M1, M2;

    for (M1 = 1, M2 = (N >> 1); M2 > 0; M1 <<= 1, M2 >>= 1) {

        if (val & M1) {

            inv |= M2;

        }

    }

    return inv;

}



void fill_w() {

    Uint16 i;

    for (i = 0; i < N; i++) {

        float angle = 2 * M_PI * i / N;

        W_R[i] = cos(angle);

        W_I[i] = -sin(angle);

    }

}



void Suma_Real() {

    Uint16 i;

    for ( i = 0; i < N; i += 2) {

        Y_R[i] = X[i] + X[i + 1];

        Y_R[i + 1] = X[i] - X[i + 1];

        Y_I[i] = 0.0;

        Y_I[i + 1] = 0.0;

    }

}



void Mult_w(Uint16 salto) {

    Uint16 s = salto >> 1;

    Uint16 j, k, r;

    for ( j = s; j < N; j += 2 * s) {

        for ( k = 0; k < s; k++) {

            r = (N * k) / salto;

            float tmp1 = Y_R[j + k] * W_R[r] - Y_I[j + k] * W_I[r];

            float tmp2 = Y_R[j + k] * W_I[r] + Y_I[j + k] * W_R[r];

            Y_R[j + k] = tmp1;

            Y_I[j + k] = tmp2;

        }

    }

}



void Suma_comp(Uint16 salto) {

    Uint16 s = salto >> 1;

    Uint16 k,m,j,i;

    for ( i = 0; i < N; i += salto) {

        for ( m = i, k = 0; k < s; k++, m++) {

            j = m + s;

            float tmp1 = Y_R[m] + Y_R[j];

            float tmp2 = Y_I[m] + Y_I[j];

            Y_R[j] = Y_R[m] - Y_R[j];

            Y_I[j] = Y_I[m] - Y_I[j];

            Y_R[m] = tmp1;

            Y_I[m] = tmp2;

        }

    }

}



///////////////////////////////////////MFCC



void compute_mfcc() {

    apply_mel_filters(power_spectrum, log_energy);

    compute_dct(log_energy);

}



void init_mel_filter_bank(float Fs) {

    float Nyquist = Fs / 2.0;

    float mel_low = 0.0;

    float mel_high = 2595.0 * log10f(1.0 + Nyquist / 700.0);  // Convertir Hz a Mel 2840

    Uint16 i;



    // Puntos equidistantes en escala Mel

    float mel_points[N_MEL_FILTERS + 2];

    for (i = 0; i < N_MEL_FILTERS + 2; i++) {

        mel_points[i] = mel_low + (mel_high - mel_low) * i / (N_MEL_FILTERS + 1);

    }



    // Convertir Mel a Hz

    float hz_points[N_MEL_FILTERS + 2];

    for (i = 0; i < N_MEL_FILTERS + 2; i++) {

        hz_points[i] = 700.0 * (powf(10.0, mel_points[i] / 2595.0) - 1.0);

    }



    // Crear filtros triangulares

    for (i = 0; i < N_MEL_FILTERS; i++) {

        float left = hz_points[i];

        float center = hz_points[i + 1];

        float right = hz_points[i + 2];

        Uint16 bin;



        for (bin = 0; bin < N/2; bin++) {

            float freq_bin = (bin * Fs) / N;

            if (freq_bin < left || freq_bin > right) {

                mel_filters[i][bin] = 0.0;

            } else if (freq_bin <= center) {

                mel_filters[i][bin] = (freq_bin - left) / (center - left);

            } else {

                mel_filters[i][bin] = (right - freq_bin) / (right - center);

            }

        }

    }

}



void apply_mel_filters() {

    Uint16 i;

    for (i = 0; i < N_MEL_FILTERS; i++) {

        log_energy[i] = 0.0;

        double energy = 0.0000000000000000;

        Uint32 bin;

        for (bin = 0; bin < N/2; bin++) {

            energy += power_spectrum[bin] * mel_filters[i][bin];

        }

        log_energy[i] = logf(energy+ 1e-6);  // Evitar log(0)

    }

}



void compute_dct(const float log_energy[N_MEL_FILTERS]) {

    Uint16 c;

    for (c = 0; c < N_CEPS; c++) {

        mfcc[c] = 0.0;

        Uint16 j;

        for (j = 0; j < N_MEL_FILTERS; j++) {

            mfcc[c] += log_energy[j] * cosf((c * (j + 0.5) * M_PI / N_MEL_FILTERS));

        }

    }

}



/////////////////////////////////Timer



Int16 CSL_gptIntr(void)

{

    CSL_Status    status;

    CSL_Config    hwConfig;

    CSL_GptObj    gptObj;



    status   = 0;



    hGpt = GPT_open (GPT_0, &gptObj, &status);

    if((NULL == hGpt) || (CSL_SOK != status)) {

        printf("GPT Open Failed\n");

    }



    status = GPT_reset(hGpt);

    if(CSL_SOK != status) {

        printf("GPT Reset Failed\n");

    }



    IRQ_clearAll();

    IRQ_disableAll();

    IRQ_setVecs((Uint32)(&VECSTART));

    IRQ_plug(TINT_EVENT, &gpt0Isr);

    IRQ_enable(TINT_EVENT);



    // Configurar GPT para 16 kHz

    hwConfig.autoLoad     = GPT_AUTO_ENABLE;

    hwConfig.ctrlTim      = GPT_TIMER_ENABLE;

    hwConfig.preScaleDiv  = GPT_PRE_SC_DIV_0; // Prescaler de 2

    hwConfig.prdLow       = (100000 / 32) - 1; // Periodo ajustado

    hwConfig.prdHigh      = 0x0000;



    status =  GPT_config(hGpt, &hwConfig);

    if(CSL_SOK != status) {

        printf("GPT Config Failed\n");

    }



    IRQ_globalEnable();

    GPT_start(hGpt);

    return 0;

}



interrupt void gpt0Isr(void)

{

    GPT_stop(hGpt);

    Hit=1;

    /* Read 16-bit left channel Data */

    EZDSP5535_I2S_readLeft(&data1);



    IRQ_clear(TINT_EVENT);



    CSL_SYSCTRL_REGS->TIAFR = 0x01;

    GPT_start(hGpt);

}
