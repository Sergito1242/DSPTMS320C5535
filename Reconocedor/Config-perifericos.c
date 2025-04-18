#include "ezdsp5535.h"
#include "ezdsp5535_gpio.h"
#include "ezdsp5535_i2c.h"
#include "ezdsp5535_i2s.h"
#include "stdio.h"

#include "csl_i2s.h"

#include <csl_mmcsd.h>
#include <csl_intc.h>
#include <csl_general.h>
#include <soc.h>

/* SD card Buffer size in Bytes                                              */
#define BUFFER_MAX_SIZE    (1024u)
/* SD card physical address with respect to sector number                    */
#define CARD_START_ADDR    (0x0)

/* Macros used to calculate system clock from PLL configurations             */
#define CSL_PLL_DIV_000    (0)
#define CSL_PLL_DIV_001    (1u)
#define CSL_PLL_DIV_002    (2u)
#define CSL_PLL_DIV_003    (3u)
#define CSL_PLL_DIV_004    (4u)
#define CSL_PLL_DIV_005    (5u)
#define CSL_PLL_DIV_006    (6u)
#define CSL_PLL_DIV_007    (7u)
#define CSL_PLL_CLOCKIN    (32768u)

#define CSL_SD_CLOCK_MAX_KHZ      (20000u)
#define AIC3204_I2C_ADDR 0x18

CSL_MMCControllerObj    pMmcsdContObj;
CSL_MmcsdHandle         mmcsdHandle;
CSL_MMCCardObj          mmcCardObj;
CSL_MMCCardIdObj        sdCardIdObj;
CSL_MMCCardCsdObj       sdCardCsdObj;

CSL_Status   mmcStatus;
Uint32       cardStatus;

Uint32 cardAddr;

CSL_Status CSL_sdConfig();
CSL_Status CSL_sdClose();

Uint32 getSysClk(void);
Uint16 computeClkRate(void);
Int16 preprocess_sample(Int16 raw_sample);
Int16 AIC3204_rset( Uint16 regnum, Uint16 regval);
Int16 AIC3204_rset( Uint16 regnum, Uint16 regval );
Int16 Configuracion( );
void Close(void);


void aic3204_Config(void)
{
    /* Configure AIC3204 */
    AIC3204_rset(0, 0x00);   // Pagina 0
    AIC3204_rset(1, 0x01);   // Reset del codec
    EZDSP5535_waitusec(1000);
    AIC3204_rset(0, 0x01);   // Pagina 1
    AIC3204_rset(1, 0x08);   // AVDD desde LDO (no desde DVDD)
    AIC3204_rset(2, 0x01);   // Habilitar bloques analógicos
    AIC3204_rset(123, 0x05); // Encender referencia analógica
    EZDSP5535_waitusec(100000); // Espera 100 ms (para estabilidad)
    AIC3204_rset(0, 0x00);   // Pagina 0

    // -----------------------------------------------
    // Configuración de PLL y Divisores para 16 kHz
    // -----------------------------------------------
    AIC3204_rset(4, 0x03);   // PLL habilitado, CLKIN = MCLK (12 MHz)
    AIC3204_rset(6, 0x08);   // J = 8
    AIC3204_rset(7, 0x02);   // D = 512 (High Byte: 0x0200)
    AIC3204_rset(8, 0x00);   // D = 512 (Low Byte)
    AIC3204_rset(5, 0x91);   // Encender PLL (P=1, R=1)
    EZDSP5535_waitusec(20000); // Esperar 20 ms

    // Divisores DAC (16 kHz)
    AIC3204_rset(11, 0x84);  // NDAC = 4 (Power up + valor)
    AIC3204_rset(12, 0x8C);  // MDAC = 12
    AIC3204_rset(13, 0x00);  // DOSR High = 64
    AIC3204_rset(14, 0x40);  // DOSR Low = 64

    // Divisores ADC (16 kHz)
    AIC3204_rset(18, 0x84);  // NADC = 4
    AIC3204_rset(19, 0x8C);  // MADC = 12
    AIC3204_rset(20, 0x40);  // AOSR = 64

    // -------------------------------------------------
    // Configuración de I2S (BCLK y WCLK para 16 kHz)
    // -------------------------------------------------
    AIC3204_rset(27, 0x0D);  // Master mode, BCLK y WCLK como salidas
    AIC3204_rset(30, 0x88);  // 32 bits por trama (16 bits por canal)

    // -----------------------------------------------
    // Ruteo y Ganancia del DAC
    // -----------------------------------------------
    AIC3204_rset(0, 0x01);   // Pagina 1
    AIC3204_rset(12, 0x08);  // LDAC → HPL
    AIC3204_rset(13, 0x08);  // RDAC → HPR
    AIC3204_rset(0, 0x00);   // Pagina 0
    AIC3204_rset(64, 0x02);  // Volumen izquierdo = derecho
    AIC3204_rset(65, 0x30);  // Ganancia DAC a +3 dB (0x20 = +3dB)
    AIC3204_rset(63, 0xD4);  // Formato I2S, 16 bits, DACs encendidos
    AIC3204_rset(0, 0x01);   // Pagina 1
    AIC3204_rset(16, 0x00);  // HPL a 0 dB (0x08 = 0 dB)
    AIC3204_rset(17, 0x00);  // HPR a 0 dB
    AIC3204_rset(9, 0x30);   // Encender HPL y HPR

    // -----------------------------------------------
    // Ruteo y Ganancia del ADC
    // -----------------------------------------------
    AIC3204_rset(0, 0x01);   // Pagina 1
    AIC3204_rset(52, 0x30);  // IN2_L → LADC_P (Micrófono)
    AIC3204_rset(55, 0x30);  // IN2_R → RADC_P
    AIC3204_rset(54, 0x03);  // CM_1 → LADC_M
    AIC3204_rset(57, 0xC0);  // CM_1 → RADC_M
    AIC3204_rset(51, 0x48);  // BIAS de micrófono a 2.5V
    AIC3204_rset(59, 0x48);  // MIC_PGA_L a 24 dB (0x18 = 24 dB)
    AIC3204_rset(60, 0x48);  // MIC_PGA_R a 24 dB
    AIC3204_rset(0, 0x00);   // Pagina 0
    AIC3204_rset(81, 0xC0);  // Encender ADC
    AIC3204_rset(82, 0x00);  // Desmutear ADC
    EZDSP5535_waitusec(100 );  // Wait
    /* Initialize I2S */

    EZDSP5535_I2S_init();
    printf("AIC configurado");
}

void Close()
{
    CSL_sdClose();
    EZDSP5535_I2S_close();    // Disble I2S
    AIC3204_rset( 1, 0x01 );  // Reset codec
}

CSL_Status CSL_sdConfig()
{
    Uint16       count;
    Uint16       actCard;
    Uint32       sectCount;
    Uint16       clockDiv;
    Uint16       rca;


    sectCount = 0;

    /* Initialize data buffers */

    /* Get the clock divider value for the current CPU frequency */
    clockDiv = computeClkRate();

    /* Initialize the CSL MMCSD module */
    CSL_Status mmcStatus = MMC_init();
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_init Failed\n");
        return(mmcStatus);
    }

    /* Open the MMCSD module in POLLED mode */

#ifdef C5515_EZDSP
    mmcsdHandle = MMC_open(&pMmcsdContObj, CSL_MMCSD1_INST, CSL_MMCSD_OPMODE_POLLED, &mmcStatus);
#else
    mmcsdHandle = MMC_open(&pMmcsdContObj, CSL_MMCSD0_INST, CSL_MMCSD_OPMODE_POLLED, &mmcStatus);
#endif
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_open Failed\n");
        return(mmcStatus);
    }
    else
    {
        printf("API: MMC_open Successful\n");
    }

    /* Send CMD0 to the card */
    mmcStatus = MMC_sendGoIdle(mmcsdHandle);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_sendGoIdle Failed\n");
        return(mmcStatus);
    }

    /* Check for the card */
    mmcStatus = MMC_selectCard(mmcsdHandle, &mmcCardObj);
    if((mmcStatus == CSL_ESYS_BADHANDLE) || (mmcStatus == CSL_ESYS_INVPARAMS))
    {
        printf("API: MMC_selectCard Failed\n");
        return(mmcStatus);
    }

    /* Verify whether the SD card is detected or not */
    if(mmcCardObj.cardType == CSL_SD_CARD)
    {
        printf("SD card Detected!\n");

        /* Check if the card is high capacity card */
        if(mmcsdHandle->cardObj->sdHcDetected == TRUE)
        {
            printf("SD card is High Capacity Card\n");
            printf("Memory Access will use Block Addressing\n\n");

            /* For the SDHC card Block addressing will be used.
               Sector address will be same as sector number */
            cardAddr = sectCount;
        }
        else
        {
            printf("SD card is Standard Capacity Card\n");
            printf("Memory Access will use Byte Addressing\n\n");

            /* For the SD card Byte addressing will be used.
               Sector address will be product of  sector number
               and sector size */
            cardAddr = (sectCount)*(CSL_MMCSD_BLOCK_LENGTH);
        }
    }
    else
    {
        /* Check if No card is inserted */
        if(mmcCardObj.cardType == CSL_CARD_NONE)
        {
            printf("No Card Detected!\n");
        }
        else
        {
            printf("SD card is not Detected!\n");
        }

        printf("Please Insert SD card!!\n");
        return(CSL_ESYS_FAIL);
    }

    /* Set the init clock */
    mmcStatus = MMC_sendOpCond(mmcsdHandle, 100);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_sendOpCond Failed\n");
        return(mmcStatus);
    }

    /* Send the card identification Data */
    mmcStatus = SD_sendAllCID(mmcsdHandle, &sdCardIdObj);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: SD_sendAllCID Failed\n");
        return(mmcStatus);
    }

    /* Set the Relative Card Address */
    mmcStatus = SD_sendRca(mmcsdHandle, &mmcCardObj, &rca);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: SD_sendRca Failed\n");
        return(mmcStatus);
    }

    /* Read the SD Card Specific Data */
    mmcStatus = SD_getCardCsd(mmcsdHandle, &sdCardCsdObj);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: SD_getCardCsd Failed\n");
        return(mmcStatus);
    }

    /* Set bus width - Optional */
    mmcStatus = SD_setBusWidth(mmcsdHandle, 1);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: SD_setBusWidth Failed\n");
        return(mmcStatus);
    }

    /* Disable SD card pull-up resistors - Optional */
    mmcStatus = SD_configurePullup(mmcsdHandle, 0);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: SD_configurePullup Failed\n");
        return(mmcStatus);
    }

    /* Set the card type in internal data structures */
    mmcStatus = MMC_setCardType(&mmcCardObj, mmcCardObj.cardType);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_setCardType Failed\n");
        return(mmcStatus);
    }

    /* Set the card pointer in internal data structures */
    mmcStatus = MMC_setCardPtr(mmcsdHandle, &mmcCardObj);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_setCardPtr Failed\n");
        return(mmcStatus);
    }

    /* Get the number of cards */
    mmcStatus = MMC_getNumberOfCards(mmcsdHandle, &actCard);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_getNumberOfCards Failed\n");
        return(mmcStatus);
    }

    /* Set clock for read-write access */
    mmcStatus = MMC_sendOpCond(mmcsdHandle, clockDiv);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_sendOpCond Failed\n");
        return(mmcStatus);
    }

    /* Set Endian mode for read and write operations */
    mmcStatus = MMC_setEndianMode(mmcsdHandle, CSL_MMCSD_ENDIAN_LITTLE,
                                  CSL_MMCSD_ENDIAN_LITTLE);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_setEndianMode Failed\n");
        return(mmcStatus);
    }

    /* Set block length for the memory card
     * For high capacity cards setting the block length will have
     * no effect
     */
    mmcStatus = MMC_setBlockLength(mmcsdHandle, CSL_MMCSD_BLOCK_LENGTH);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_setBlockLength Failed\n");
        return(mmcStatus);
    }

    return mmcStatus;
}



CSL_Status CSL_sdClose()
{
    /* Get card stataus */
    mmcStatus = MMC_getCardStatus(mmcsdHandle, &cardStatus);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_getCardStatus Failed\n");
        return(mmcStatus);
    }

    /* Deselect the SD card */
    mmcStatus = MMC_deselectCard(mmcsdHandle, &mmcCardObj);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_deselectCard Failed\n");
        return(mmcStatus);
    }

    /* Clear the MMCSD card response registers */
    mmcStatus = MMC_clearResponse(mmcsdHandle);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_clearResponse Failed\n");
        return(mmcStatus);
    }

    /* Send CMD0 to the SD card */
    mmcStatus = MMC_sendCmd(mmcsdHandle, 0x00, 0x00, 0xFFFF);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_sendCmd Failed\n");
        return(mmcStatus);
    }

    /* Close the MMCSD module */
    mmcStatus = MMC_close(mmcsdHandle);
    if(mmcStatus != CSL_SOK)
    {
        printf("API: MMC_close Failed\n");
        return(mmcStatus);
    }
    else
    {
        printf("API: MMC_close Successful\n");
    }
    return mmcStatus;
}

Uint16 computeClkRate(void)
{
    Uint32    sysClock;
    Uint32    remainder;
    Uint32    memMaxClk;
    Uint16    clkRate;

    sysClock  = 0;
    remainder = 0;
    memMaxClk = CSL_SD_CLOCK_MAX_KHZ;
    clkRate   = 0;

    /* Get the clock value at which CPU is running */
    sysClock = getSysClk();

    if (sysClock > memMaxClk)
    {
        if (memMaxClk != 0)
        {
            clkRate   = sysClock / memMaxClk;
            remainder = sysClock % memMaxClk;

            /*
             * If the remainder is not equal to 0, increment clock rate to make
             * sure that memory clock value is less than the value of
             * 'CSL_SD_CLOCK_MAX_KHZ'.
             */
            if (remainder != 0)
            {
                clkRate++;
            }

            /*
             * memory clock divider '(2 * (CLKRT + 1)' will always
             * be an even number. Increment the clock rate in case of
             * clock rate is not an even number
             */
            if (clkRate%2 != 0)
            {
                clkRate++;
            }

            /*
             * AT this point 'clkRate' holds the value of (2 * (CLKRT + 1).
             * Get the value of CLKRT.
             */
            clkRate = clkRate/2;
            clkRate = clkRate - 1;

            /*
             * If the clock rate is more than the maximum allowed clock rate
             * set the value of clock rate to maximum value.
             * This case will become true only when the value of
             * 'CSL_SD_CLOCK_MAX_KHZ' is less than the minimum possible
             * memory clock that can be generated at a particular CPU clock.
             *
             */
            if (clkRate > CSL_MMC_MAX_CLOCK_RATE)
            {
                clkRate = CSL_MMC_MAX_CLOCK_RATE;
            }
        }
        else
        {
            clkRate = CSL_MMC_MAX_CLOCK_RATE;
        }
    }

    return (clkRate);
}

/**
 *  \brief  Function to calculate the clock at which system is running
 *
 *  \param    none
 *
 *  \return   System clock value in KHz
 */

#if (defined(CHIP_C5505_C5515) || defined(CHIP_C5504_C5514))

Uint32 getSysClk(void)
{
    Bool      pllRDBypass;
    Bool      pllOutDiv;
    Uint32    sysClk;
    Uint16    pllVP;
    Uint16    pllVS;
    Uint16    pllRD;
    Uint16    pllVO;

    pllVP = CSL_FEXT(CSL_SYSCTRL_REGS->CGCR1, SYS_CGCR1_VP);
    pllVS = CSL_FEXT(CSL_SYSCTRL_REGS->CGCR1, SYS_CGCR1_VS);

    pllRD = CSL_FEXT(CSL_SYSCTRL_REGS->CGICR, SYS_CGICR_RDRATIO);
    pllVO = CSL_FEXT(CSL_SYSCTRL_REGS->CGOCR, SYS_CGOCR_OD);

    pllRDBypass = CSL_FEXT(CSL_SYSCTRL_REGS->CGICR, SYS_CGICR_RDBYPASS);
    pllOutDiv   = CSL_FEXT(CSL_SYSCTRL_REGS->CGOCR, SYS_CGOCR_OUTDIVEN);

    sysClk = CSL_PLL_CLOCKIN;

    if (0 == pllRDBypass)
    {
        sysClk = sysClk/(pllRD + 4);
    }

    sysClk = (sysClk * ((pllVP << 2) + pllVS + 4));

    if (1 == pllOutDiv)
    {
        sysClk = sysClk/(pllVO + 1);
    }

    /* Return the value of system clock in KHz */
    return(sysClk/1000);
}

#else

Uint32 getSysClk(void)
{
    Bool      pllRDBypass;
    Bool      pllOutDiv;
    Bool      pllOutDiv2;
    Uint32    sysClk;
    Uint16    pllVP;
    Uint16    pllVS;
    Uint16    pllRD;
    Uint16    pllVO;
    Uint16    pllDivider;
    Uint32    pllMultiplier;

    pllVP = CSL_FEXT(CSL_SYSCTRL_REGS->CGCR1, SYS_CGCR1_MH);
    pllVS = CSL_FEXT(CSL_SYSCTRL_REGS->CGICR, SYS_CGICR_ML);

    pllRD = CSL_FEXT(CSL_SYSCTRL_REGS->CGICR, SYS_CGICR_RDRATIO);
    pllVO = CSL_FEXT(CSL_SYSCTRL_REGS->CGOCR, SYS_CGOCR_ODRATIO);

    pllRDBypass = CSL_FEXT(CSL_SYSCTRL_REGS->CGICR, SYS_CGICR_RDBYPASS);
    pllOutDiv   = CSL_FEXT(CSL_SYSCTRL_REGS->CGOCR, SYS_CGOCR_OUTDIVEN);
    pllOutDiv2  = CSL_FEXT(CSL_SYSCTRL_REGS->CGOCR, SYS_CGOCR_OUTDIV2BYPASS);

    pllDivider = ((pllOutDiv2) | (pllOutDiv << 1) | (pllRDBypass << 2));

    pllMultiplier = ((Uint32)CSL_PLL_CLOCKIN * ((pllVP << 2) + pllVS + 4));

    switch(pllDivider)
    {
        case CSL_PLL_DIV_000:
        case CSL_PLL_DIV_001:
            sysClk = pllMultiplier / (pllRD + 4);
        break;

        case CSL_PLL_DIV_002:
            sysClk = pllMultiplier / ((pllRD + 4) * (pllVO + 4) * 2);
        break;

        case CSL_PLL_DIV_003:
            sysClk = pllMultiplier / ((pllRD + 4) * 2);
        break;

        case CSL_PLL_DIV_004:
        case CSL_PLL_DIV_005:
            sysClk = pllMultiplier;
        break;

        case CSL_PLL_DIV_006:
            sysClk = pllMultiplier / ((pllVO + 4) * 2);
        break;

        case CSL_PLL_DIV_007:
            sysClk = pllMultiplier / 2;
        break;
    }

    /* Return the value of system clock in KHz */
    return(sysClk/1000);
}
#endif


Int16 AIC3204_rget(  Uint16 regnum, Uint16* regval )
{
    Int16  retcode = 0;
    Uint16 cmd[2];

    cmd[0] = regnum & 0x007F;       // 7-bit Device Register
    cmd[1] = 0;

    retcode |= EZDSP5535_I2C_write( AIC3204_I2C_ADDR, cmd, 1 );
    retcode |= EZDSP5535_I2C_read( AIC3204_I2C_ADDR, cmd, 1 );

    *regval = cmd[0];
    EZDSP5535_wait( 10 );
    return retcode;
}


Int16 AIC3204_rset( Uint16 regnum, Uint16 regval )
{
    Uint16 cmd[2];
    cmd[0] = regnum & 0x007F;       // 7-bit Device Register
    cmd[1] = regval;                // 8-bit Register Data

    EZDSP5535_waitusec( 300 );

    return EZDSP5535_I2C_write( AIC3204_I2C_ADDR, cmd, 2 );
}


Int16 Configuracion( )
{
    /* Initialize I2C */
    EZDSP5535_I2C_init( );
    aic3204_Config();
    CSL_sdConfig();

    return 0;
}
