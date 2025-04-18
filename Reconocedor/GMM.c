#include "stdio.h"
#include "ezdsp5535.h"
#include "ezdsp5535_sar.h"
#include <math.h>
#include <float.h>
#include <string.h>  // Para memcpy

#define DIM 13            // Coeficientes MFCC por ventana
#define MAX_COMP 3       // Máximo número de componentes
#define LOG_2PI 1.837877  // log(2*PI)
#define MAX_TEMPLATE_SIZE 78

typedef struct {
    Uint16 n_componentes;
    double pesos[MAX_COMP];
    double medias[MAX_COMP][DIM];
    double covarianzas[MAX_COMP][DIM];
} ModeloGMM;

extern double Voice_Mfcc[MAX_TEMPLATE_SIZE][DIM];

double result;

ModeloGMM Comandos[5];



void inicializarGMM()
{
    // Inicialización para Comandos[0]
    Comandos[0].n_componentes = 3;

    double pesos1[3] = {0.1828, 0.6590, 0.1583 };
    memcpy(Comandos[0].pesos, pesos1, sizeof(pesos1));

    double medias1[3][DIM] = {
      { -146.6137, -24.9267, 2.4589, -2.8702, 1.5030, -0.3633, -0.6030, -2.8272, 1.0721, -2.4018, -0.3030, 0.7295, -0.5420 },
      { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 },
      { -168.7738, -1.3417, 3.7352, -3.3762, 3.5811, -5.5365, -4.3196, -5.0976, -2.7613, -2.3875, -1.7145, -0.2381, -0.3009 },
      };
    memcpy(Comandos[0].medias, medias1, sizeof(medias1));

    double covariancias1[3][DIM] = {
     { 74.0031, 98.9809, 51.6717, 10.8772, 13.9000, 6.6553, 4.2574, 10.6338, 8.4233, 5.9336, 3.1593, 1.8869, 1.4515 },
     { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 },
     { 82.7452, 18.8100, 10.8446, 12.7595, 38.7018, 5.7313, 8.2143, 10.3953, 6.0408, 7.2914, 6.1744, 2.9037, 3.2009 },
     };
    memcpy(Comandos[0].covarianzas, covariancias1, sizeof(covariancias1));

    // Inicialización para Comandos[1]
    Comandos[1].n_componentes = 3;

    double pesos2[3] = {0.5718, 0.2444, 0.1838};
    memcpy(Comandos[1].pesos, pesos2, sizeof(pesos2));


    double medias2[3][DIM] = {
         { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
           0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 },  // Componente 0
         { -163.7418, -2.5816, 3.9353, -3.7022, 0.2993, -6.2201, -3.8964,
           -3.6331, 0.6640, 0.1997, 1.8641, 0.5233, -1.0045 }, // Componente 1
         { -169.6332, -20.1500, 4.2277, 1.8737, 1.5287, 0.3067, 1.0805,
           0.9260, 1.7609, -0.0495, 1.6095, 1.1319, 0.1576 }   // Componente 2
    };
    memcpy(Comandos[1].medias, medias2, sizeof(medias2));

    double covariancias2[3][DIM] = {
         { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
           0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 },  // Componente 0
         { 136.2564, 34.2453, 9.3944, 35.3116, 9.0379, 6.7575, 5.1927,
           5.0266, 6.4255, 3.9453, 3.1734, 2.3838, 3.7140 }, // Componente 1
         { 226.8084, 202.5688, 14.1774, 14.4647, 5.8301, 3.7117, 2.9356,
           3.6327, 2.5955, 3.0406, 1.9710, 1.5177, 2.2335 }   // Componente 2
    };
    memcpy(Comandos[1].covarianzas, covariancias2, sizeof(covariancias2));

    // Inicialización para Comandos[2]
    Comandos[2].n_componentes = 3;

    double pesos3[3] = {0.1102, 0.4308, 0.4591};
    memcpy(Comandos[2].pesos, pesos3, sizeof(pesos3));

    double medias3[3][DIM] = {
      { -165.5469, -6.3302, 1.2139, -1.3029, -0.2093, -1.5280, -0.5451, -0.1033, 0.7914, 0.1728, 1.3923, 0.4116, -0.0527 },
      { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 },
      { -163.0569, -0.5241, 0.5556, -11.9168, -3.1213, -8.5754, -2.8614, -2.8159, 1.9447, 0.8631, 2.8779, -1.4791, -1.7060 },
      };
    memcpy(Comandos[2].medias, medias3, sizeof(medias3));

    double covarianzas3[3][DIM] = {
       { 153.1733, 12.0011, 6.6714, 8.1541, 5.3757, 5.7280, 3.3581, 4.0494, 1.9925, 3.3612, 1.9150, 1.4553, 1.6589 },
       { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 },
       { 75.9291, 13.5070, 6.8839, 14.3013, 11.6089, 8.9006, 7.4323, 4.1090, 4.6325, 3.0383, 2.4973, 3.5557, 3.0423 },
       };
    memcpy(Comandos[2].covarianzas, covarianzas3, sizeof(covarianzas3));

    // Inicialización para Comandos[3]
    Comandos[3].n_componentes = 3;

    double pesos4[3] = {0.4718, 0.4110, 0.1172 };
    memcpy(Comandos[3].pesos, pesos4, sizeof(pesos3));

    double medias4[3][DIM] = {
        { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 },
        { -157.5237, -1.7178, -2.6269, -14.6082, -1.7281, -6.9410, -0.6773, -2.4568, 2.5308, 2.3208, 1.1002, -3.6721, -2.0070 },
        { -154.4123, -8.1238, -0.6361, -4.2039, -1.8027, -0.6184, 0.3491, -0.4347, 1.3157, 0.8258, 0.3592, -1.0792, -0.2399 },
        };
    memcpy(Comandos[3].medias, medias4, sizeof(medias4));

    double covarianzas4[3][DIM] = {
       { 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 },
       { 62.3299, 16.8188, 7.7400, 11.6228, 13.8231, 7.9374, 6.5012, 3.7536, 5.3210, 4.0191, 2.0350, 3.1921, 2.1394 },
       { 179.6538, 22.1819, 6.2355, 15.4133, 10.9732, 3.2302, 4.2022, 4.7464, 3.4723, 3.2707, 2.4731, 3.6455, 1.9114 },
       };
    memcpy(Comandos[3].covarianzas, covarianzas4, sizeof(covarianzas4));
    // Inicialización para Comandos[4]
    Comandos[4].n_componentes = 3;

    double pesos5[3] = {0.3608, 0.2577, 0.3815 };
    memcpy(Comandos[4].pesos, pesos5, sizeof(pesos5));

    double medias5[3][DIM] = {
      { -166.3777, -0.6132, 2.2640, -10.0821, -5.1533, -6.7445, -2.2840, -5.2540, 2.8284, 1.0889, 4.5588, -0.4005, -1.5768 },
      { 0.0381, -0.0866, -0.0708, -7.1691, 0.2159, 0.3454, -0.1607, -0.0392, -0.2327, -0.0355, -0.0730, 0.0645, -0.0578 },
      { -165.3678, -4.8384, 3.1510, -0.3567, -0.3513, -5.3904, -4.0507, -2.0819, 1.3195, -0.8372, 0.2660, 0.6464, -0.4465 },
    };
    memcpy(Comandos[4].medias, medias5, sizeof(medias5));

    double covarianzas5[3][DIM] = {
       { 82.0487, 14.8036, 8.0378, 9.4141, 15.1836, 5.0286, 3.2408, 4.3302, 6.7509, 2.3376, 3.5941, 3.6886, 2.6546 },
       { 0.0996, 0.2235, 0.1653, 1245.2085, 1.8368, 3.2621, 0.6766, 0.1636, 1.4037, 0.0596, 0.2273, 0.1345, 0.1049 },
       { 258.2517, 21.4627, 8.3527, 15.1671, 12.8249, 21.7950, 15.5626, 5.2566, 3.2542, 5.7140, 3.9417, 2.8562, 2.5632 },
    };
    memcpy(Comandos[4].covarianzas, covarianzas5, sizeof(covarianzas5));
}

void inicializar2()
{
    // Inicialización para Comandos[2]
    Comandos[2].n_componentes = 3;

    double pesos2[3] = {0.7436, 0.1507, 0.1057};
    memcpy(Comandos[2].pesos, pesos2, sizeof(pesos2));

    double medias2[3][DIM] = {
         {  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
            0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000 },  // Componente 0
         { -167.1975, -0.6420,  3.6293, -7.1161, -1.1133, -6.7627, -1.9218,
           -4.0344, -0.2452, -0.7502,  3.1508,  1.8194,  0.1113 },  // Componente 1
         { -179.1945, -11.3455,  1.1770,  1.1946,  2.8130, -0.4472,  1.2513,
            1.7752,  0.7972,  0.6228,  1.4751,  1.3873,  0.5284 }   // Componente 2
    };
    memcpy(Comandos[2].medias, medias2, sizeof(medias2));

    double covariancias2[3][DIM] = {
         {  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
            0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000 },  // Componente 0
         { 224.4716, 40.0715, 13.3849, 12.4547, 13.0935,  6.6755,  4.9969,
            4.7883,  3.7832,  5.4266,  2.6945,  2.3861,  5.3715 },  // Componente 1
         { 214.4173, 16.6497,  7.3301,  8.1312,  5.6733,  4.0413,  2.4666,
            3.2516,  1.5798,  1.1135,  2.4327,  2.1968,  2.1321 }   // Componente 2
    };
    memcpy(Comandos[2].covarianzas, covariancias2, sizeof(covariancias2));

}


///////////////////////////////////////////////////////////////////////////
double logsumexp(double logprobs[3], Uint16 n) {
    double max = -DBL_MAX;
    Uint16 i;
    for(i = 0; i < n; i++) {
        if(logprobs[i] > max) max = logprobs[i];
    }

    double sum = 0.0;
    for(i = 0; i < n; i++) {
        sum += exp(logprobs[i] - max);
    }

    return max + log(sum);
}

double log_probabilidad_ventana(ModeloGMM modelo, double ventana[DIM]) {
    double log_probs[MAX_COMP];
    Uint16 i, d;

    for(i = 0; i < modelo.n_componentes; i++) {
        double log_peso = log(modelo.pesos[i]);
        double termino_mahal = 0.0;
        double termino_cov = 0.0;

        for(d = 0; d < DIM; d++) {
            double diff = ventana[d] - modelo.medias[i][d];
            termino_mahal += (diff * diff) / modelo.covarianzas[i][d];
            termino_cov += log(modelo.covarianzas[i][d]);
        }

        log_probs[i] = log_peso - 0.5 * (termino_mahal + termino_cov + DIM * LOG_2PI);
    }

    return logsumexp(log_probs, modelo.n_componentes);
}

double Probabilidad_GMM(Uint16 Num_Comando, Uint16 num_ventanas)
{
    if(num_ventanas == 0) return -DBL_MAX;

    double log_prob_total = 0.0;
    Uint16 v;

    for(v = 0; v < num_ventanas; v++) {
        log_prob_total += log_probabilidad_ventana(Comandos[Num_Comando], Voice_Mfcc[v]);
    }

    result = log_prob_total;
    return result;
}



