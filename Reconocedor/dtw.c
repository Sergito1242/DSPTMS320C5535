#include <stdlib.h>
#include <float.h>
#include <stdint.h>
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

#define MAX_TEMPLATE_SIZE 78
#define MAX_MFCC_COEFS 13

// Matrices externas
extern double Voice_Mfcc[MAX_TEMPLATE_SIZE][MAX_MFCC_COEFS];
extern double Templest[5][MAX_TEMPLATE_SIZE][MAX_MFCC_COEFS];
double result;

// Función para calcular la distancia euclidiana
double euclidean_distance(double vec1[MAX_MFCC_COEFS], double vec2[MAX_MFCC_COEFS], Uint16 size) {
    double sum = 0.0;
    Uint16 i;
    for (i = 0; i < size; i++) {
        double diff = vec1[i] - vec2[i];
        sum += diff * diff;
    }
    return sqrt(sum);
}

// Implementación del algoritmo DTW
double dtw_custom(Uint16 len1, Uint16 len2, Uint16 num_coefs, Uint16 index) {
    double cost[MAX_TEMPLATE_SIZE][MAX_TEMPLATE_SIZE];
    Int16 i, j;
    double dist;
    len1 -=0;
    len2 -=0;

    // Inicializar primera celda
    cost[0][0] = euclidean_distance(Voice_Mfcc[0], Templest[index][0], num_coefs);

    // Inicializar primera columna
    for (i = 1; i < len1; i++) {
        cost[i][0] = cost[i - 1][0] + euclidean_distance(Voice_Mfcc[i], Templest[index][0], num_coefs);
    }

    // Inicializar primera fila
    for (j = 1; j < len2; j++) {
        cost[0][j] = cost[0][j - 1] + euclidean_distance(Voice_Mfcc[0], Templest[index][j], num_coefs);
    }

    // Llenar el resto de la matriz
    for (i = 1; i < len1; i++) {
        for (j = 1; j < len2; j++) {
            dist = euclidean_distance(Voice_Mfcc[i], Templest[index][j], num_coefs);

            // Buscar el mínimo sin usar fmin
            double min_cost = cost[i - 1][j];
            if (cost[i][j - 1] < min_cost) {
                min_cost = cost[i][j - 1];
            }
            if (cost[i - 1][j - 1] < min_cost) {
                min_cost = cost[i - 1][j - 1];
            }

            cost[i][j] = dist + min_cost;
        }
    }
    result = cost[len1 - 1][len2 - 1];
    return result;
}
