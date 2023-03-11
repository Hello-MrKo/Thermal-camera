/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/*
 Modified for Lower Power Arduino  -Koji Ohashi 2019.05.28-
 */
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API_UNO.h"
#include <math.h>

int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2);
int CheckEEPROMValid(uint16_t *eeData);

// ++++++++++++++++++++++
void ExtractVDDParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
void ExtractPTATParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
void ExtractGainParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
void ExtractTgcParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
void ExtractResolutionParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
void ExtractKsTaParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
void ExtractKsToParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
void ExtractCPParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
void ExtractCILCParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
int ExtractDeviatingPixelsUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
float MLX90640_GetVddUNO(uint16_t *frameParams, const paramsMLX90640UNO *SnrParamsUNO);
float MLX90640_GetTaUNO(uint16_t *frameParams, const paramsMLX90640UNO *SnrParamsUNO);


uint16_t test_eeData(uint16_t *eeData, int num){
  uint16_t value = pgm_read_word(eeData + num);
  return value;
}


int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t *eeData)
{
     return MLX90640_I2CRead(slaveAddr, 0x2400, 832, eeData);
}


int MLX90640_WaitFrameDataUNO(uint8_t slaveAddr) //for UNO
{
    uint16_t dataReady = 1;
    uint16_t statusRegister;
    int error = 1;
    dataReady = 0;
    while(dataReady == 0)
    {
        error = MLX90640_I2CRead(slaveAddr, 0x8000, 1, &statusRegister);
        if(error != 0)
        {
            return error;
        }    
        dataReady = statusRegister & 0x0008;//A new data is available in RAM
    }
    error = MLX90640_I2CWrite(slaveAddr, 0x8000, 0x0030);//test16
    return error;
}


int MLX90640_GetFrameDataUNO(uint8_t slaveAddr, int Line, uint16_t *frameData_Line)
{   
    //uint8_t slaveAddr = i2caddr;
    int error = 1;
    int startPos=Line*32;
    error = MLX90640_I2CRead(slaveAddr, 0x0400 + startPos, 32, frameData_Line);
    return error;
}


int MLX90640_GetFrameParamsUNO(uint8_t slaveAddr, uint16_t *frameParams)//uint16_t frameParams[66];
{
      uint16_t controlRegister1;
      uint16_t statusRegister;
      int error = 1;
      
      error = MLX90640_I2CRead(slaveAddr, 0x0700, 64, frameParams);
      error = MLX90640_I2CRead(slaveAddr, 0x8000, 1, &statusRegister);
      error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
      frameParams[64] = controlRegister1;
      frameParams[65] = statusRegister & 0x0001;
      return error;
}



int MLX90640_ExtractParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    int error = CheckEEPROMValid(eeData);
    
    if(error == 0)
    {
        ExtractVDDParametersUNO(eeData, mlx90640);
        
        ExtractPTATParametersUNO(eeData, mlx90640);
        
        ExtractGainParametersUNO(eeData, mlx90640);
        ExtractTgcParametersUNO(eeData, mlx90640);
        ExtractResolutionParametersUNO(eeData, mlx90640);
        ExtractKsTaParametersUNO(eeData, mlx90640);
        ExtractKsToParametersUNO(eeData, mlx90640);
        //ExtractAlphaParameters(eeData, mlx90640);
        //ExtractOffsetParameters(eeData, mlx90640);
        //ExtractKtaPixelParameters(eeData, mlx90640);
        //ExtractKvPixelParameters(eeData, mlx90640);
        ExtractCPParametersUNO(eeData, mlx90640);
        ExtractCILCParametersUNO(eeData, mlx90640);
        error = ExtractDeviatingPixelsUNO(eeData, mlx90640);
        
    }
    
    return error;

}



//------------------------------------------------------------------------------


void MLX90640_CalculateToUNO(uint8_t slaveAddr, uint16_t *frameParams, uint16_t *eeData, paramsMLX90640UNO *SnrParamsUNO, float emissivity, float ta_Shift, int Line, float *T_Line)
{
    float vdd;
    float tr;
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP[2];
    float irData;
    float alphaCompensated;
    uint8_t mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float Sx;
    float To;
    float alphaCorrR[4];
    int8_t range;
    uint16_t subPage;
    
    uint16_t frameData_Line[32];
    //uint16_t frameParams[66];
    float alpha_Line[32];
    int16_t offset_Line[32];
    float kta_Line[32];
    float kv_Line[32];

    
//------------------------- Line Data -----------------------------------
    ExtractAlphaParametersUNO(eeData, Line, alpha_Line);
    ExtractOffsetParametersUNO(eeData, Line, offset_Line);
    ExtractKtaPixelParametersUNO(eeData, Line, kta_Line);
    ExtractKvPixelParametersUNO(eeData, Line, kv_Line);
    MLX90640_GetFrameDataUNO(slaveAddr, Line, frameData_Line);
    //MLX90640_GetFrameParamsUNO(slaveAddr, frameParams);


    
//-------------------------end of Line Data -----------------------------------    
    //subPage = frameData[833];
    subPage = frameParams[833-768];
    //vdd = MLX90640_GetVdd(frameData, params);//
    vdd = MLX90640_GetVddUNO(frameParams, SnrParamsUNO);
    //ta = MLX90640_GetTa(frameData, params);
    ta = MLX90640_GetTaUNO(frameParams, SnrParamsUNO);

    tr=ta-ta_Shift;//test16
    ta4 = pow((ta + 273.15), (double)4);
    tr4 = pow((tr + 273.15), (double)4);
    taTr = tr4 - (tr4-ta4)/emissivity;
    
    alphaCorrR[0] = 1 / (1 + SnrParamsUNO->ksTo[0] * 40);
    alphaCorrR[1] = 1 ;
    alphaCorrR[2] = (1 + SnrParamsUNO->ksTo[2] * SnrParamsUNO->ct[2]);
    alphaCorrR[3] = alphaCorrR[2] * (1 + SnrParamsUNO->ksTo[3] * (SnrParamsUNO->ct[3] - SnrParamsUNO->ct[2]));
    
//------------------------- Gain calculation -----------------------------------    
    //gain = frameData[778];
    gain = frameParams[778-768];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }
    
    gain = SnrParamsUNO->gainEE / gain; 
  
//------------------------- To calculation -------------------------------------    
    //mode = (frameData[832] & 0x1000) >> 5;
    mode = (frameParams[832-768] & 0x1000) >> 5;
    
    //irDataCP[0] = frameData[776];  
    //irDataCP[1] = frameData[808];
    irDataCP[0] = frameParams[776-768];  
    irDataCP[1] = frameParams[808-768];
    for( int i = 0; i < 2; i++)
    {
        if(irDataCP[i] > 32767)
        {
            irDataCP[i] = irDataCP[i] - 65536;
        }
        irDataCP[i] = irDataCP[i] * gain;
    }
    irDataCP[0] = irDataCP[0] - SnrParamsUNO->cpOffset[0] * (1 + SnrParamsUNO->cpKta * (ta - 25)) * (1 + SnrParamsUNO->cpKv * (vdd - 3.3));
    if( mode ==  SnrParamsUNO->calibrationModeEE)
    {
        irDataCP[1] = irDataCP[1] - SnrParamsUNO->cpOffset[1] * (1 + SnrParamsUNO->cpKta * (ta - 25)) * (1 + SnrParamsUNO->cpKv * (vdd - 3.3));
    }
    else
    {
      irDataCP[1] = irDataCP[1] - (SnrParamsUNO->cpOffset[1] + SnrParamsUNO->ilChessC[0]) * (1 + SnrParamsUNO->cpKta * (ta - 25)) * (1 + SnrParamsUNO->cpKv * (vdd - 3.3));
    }

    //for( int pixelNumber = 0; pixelNumber < 768; pixelNumber++)
    
    for (int i=0; i<32; i++)
    {
        int pixelNumber=Line*32 + i;
        
        ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2; 
        chessPattern = ilPattern ^ (pixelNumber - (pixelNumber/2)*2); 
        conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);
        
        if(mode == 0)
        {
          pattern = ilPattern; 
        }
        else 
        {
          pattern = chessPattern; 
        }               
        
        //if(pattern == frameData[833])
        if(pattern == frameParams[833-768])
        {    
            irData = frameData_Line[i];
            if(irData > 32767)
            {
                irData = irData - 65536;
            }
            irData = irData * gain;
            
            irData = irData - offset_Line[i]*(1 + kta_Line[i]*(ta - 25))*(1 + kv_Line[i]*(vdd - 3.3));
            if(mode !=  SnrParamsUNO->calibrationModeEE)
            {
              irData = irData + SnrParamsUNO->ilChessC[2] * (2 * ilPattern - 1) - SnrParamsUNO->ilChessC[1] * conversionPattern; 
            }
            
            irData = irData / emissivity;
    
            irData = irData - SnrParamsUNO->tgc * irDataCP[subPage];
            
            alphaCompensated = (alpha_Line[i] - SnrParamsUNO->tgc * SnrParamsUNO->cpAlpha[subPage])*(1 + SnrParamsUNO->KsTa * (ta - 25));
            
            Sx = pow((double)alphaCompensated, (double)3) * (irData + alphaCompensated * taTr);
            Sx = sqrt(sqrt(Sx)) * SnrParamsUNO->ksTo[1];
            
            To = sqrt(sqrt(irData/(alphaCompensated * (1 - SnrParamsUNO->ksTo[1] * 273.15) + Sx) + taTr)) - 273.15;
                    
            if(To < SnrParamsUNO->ct[1])
            {
                range = 0;
            }
            else if(To < SnrParamsUNO->ct[2])   
            {
                range = 1;            
            }   
            else if(To < SnrParamsUNO->ct[3])
            {
                range = 2;            
            }
            else
            {
                range = 3;            
            }      
            
            To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + SnrParamsUNO->ksTo[range] * (To - SnrParamsUNO->ct[range]))) + taTr)) - 273.15;
            
            T_Line[i] = To;
        }
    }
}







float MLX90640_GetVddUNO(uint16_t *frameParams, const paramsMLX90640UNO *SnrParamsUNO)
{
    float vdd;
    float resolutionCorrection;

    int resolutionRAM;    
    
    //vdd = frameData[810];
    vdd = frameParams[810-768];
    if(vdd > 32767)
    {
        vdd = vdd - 65536;
    }
    //resolutionRAM = (frameData[832] & 0x0C00) >> 10;
    resolutionRAM = (frameParams[832-768] & 0x0C00) >> 10;
    resolutionCorrection = pow(2, (double)SnrParamsUNO->resolutionEE) / pow(2, (double)resolutionRAM);
    vdd = (resolutionCorrection * vdd - SnrParamsUNO->vdd25) / SnrParamsUNO->kVdd + 3.3;
    
    return vdd;
}
//------------------------------------------------------------------------------



float MLX90640_GetTaUNO(uint16_t *frameParams, const paramsMLX90640UNO *SnrParamsUNO)
{
    float ptat;
    float ptatArt;
    float vdd;
    float ta;
    
    vdd = MLX90640_GetVddUNO(frameParams, SnrParamsUNO);
    
    //ptat = frameData[800];
    ptat = frameParams[800-768];
    if(ptat > 32767)
    {
        ptat = ptat - 65536;
    }
    
    //ptatArt = frameData[768];
    ptatArt = frameParams[768-768];
    if(ptatArt > 32767)
    {
        ptatArt = ptatArt - 65536;
    }
    ptatArt = (ptat / (ptat * SnrParamsUNO->alphaPTAT + ptatArt)) * pow(2, (double)18);
    
    ta = (ptatArt / (1 + SnrParamsUNO->KvPTAT * (vdd - 3.3)) - SnrParamsUNO->vPTAT25);
    ta = ta / SnrParamsUNO->KtPTAT + 25;
    
    return ta;
}

//------------------------------------------------------------------------------

int MLX90640_GetSubPageNumber(uint16_t *frameData)
{
    return frameData[833];    

}    

//------------------------------------------------------------------------------


void ExtractVDDParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    int16_t kVdd;
    int16_t vdd25;
    uint16_t eeData_51=pgm_read_word(eeData + 51);

    kVdd = eeData_51;

    kVdd = (eeData_51 & 0xFF00) >> 8;
    if(kVdd > 127)
    {
        kVdd = kVdd - 256;
    }
    kVdd = 32 * kVdd;
    vdd25 = eeData_51 & 0x00FF;
    vdd25 = ((vdd25 - 256) << 5) - 8192;
    
    mlx90640->kVdd = kVdd;
    mlx90640->vdd25 = vdd25; 
}
//------------------------------------------------------------------------------

void ExtractPTATParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    float KvPTAT;
    float KtPTAT;
    int16_t vPTAT25;
    float alphaPTAT;
    uint16_t eeData_50=pgm_read_word(eeData + 50);
    uint16_t eeData_49=pgm_read_word(eeData + 49);
    uint16_t eeData_16=pgm_read_word(eeData + 16);
    
    KvPTAT = (eeData_50 & 0xFC00) >> 10;
    if(KvPTAT > 31)
    {
        KvPTAT = KvPTAT - 64;
    }
    KvPTAT = KvPTAT/4096;
    
    KtPTAT = eeData_50 & 0x03FF;
    if(KtPTAT > 511)
    {
        KtPTAT = KtPTAT - 1024;
    }
    KtPTAT = KtPTAT/8;
    
    vPTAT25 = eeData_49;
    
    alphaPTAT = (eeData_16 & 0xF000) / pow(2, (double)14) + 8.0f;
    
    mlx90640->KvPTAT = KvPTAT;
    mlx90640->KtPTAT = KtPTAT;    
    mlx90640->vPTAT25 = vPTAT25;
    mlx90640->alphaPTAT = alphaPTAT;   
}

//------------------------------------------------------------------------------

void ExtractGainParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    int16_t gainEE;
    uint16_t eeData_48=pgm_read_word(eeData + 48);
    
    gainEE = eeData_48;
    if(gainEE > 32767)
    {
        gainEE = gainEE -65536;
    }
    
    mlx90640->gainEE = gainEE;    
}
//------------------------------------------------------------------------------

void ExtractTgcParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    float tgc;
    uint16_t eeData_60=pgm_read_word(eeData + 60);
    tgc = eeData_60 & 0x00FF;
    if(tgc > 127)
    {
        tgc = tgc - 256;
    }
    tgc = tgc / 32.0f;
    
    mlx90640->tgc = tgc;        
}

//------------------------------------------------------------------------------

void ExtractResolutionParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    uint8_t resolutionEE;
    uint16_t eeData_56=pgm_read_word(eeData + 56);
    resolutionEE = (eeData_56 & 0x3000) >> 12;    
    
    mlx90640->resolutionEE = resolutionEE;
}
//------------------------------------------------------------------------------

void ExtractKsTaParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    float KsTa;
    uint16_t eeData_60=pgm_read_word(eeData + 60);
    KsTa = (eeData_60 & 0xFF00) >> 8;
    if(KsTa > 127)
    {
        KsTa = KsTa -256;
    }
    KsTa = KsTa / 8192.0f;
    
    mlx90640->KsTa = KsTa;
}

//------------------------------------------------------------------------------

void ExtractKsToParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    unsigned long KsToScale;//32bit
    //int KsToScale;16bit on UNO
    int8_t step;
    uint16_t eeData_63=pgm_read_word(eeData + 63);
    uint16_t eeData_61=pgm_read_word(eeData + 61);
    uint16_t eeData_62=pgm_read_word(eeData + 62);
    
    step = ((eeData_63 & 0x3000) >> 12) * 10;
    
    mlx90640->ct[0] = -40;
    mlx90640->ct[1] = 0;
    mlx90640->ct[2] = (eeData_63 & 0x00F0) >> 4;
    mlx90640->ct[3] = (eeData_63 & 0x0F00) >> 8;
    
    mlx90640->ct[2] = mlx90640->ct[2]*step;
    mlx90640->ct[3] = mlx90640->ct[2] + mlx90640->ct[3]*step;
    
    KsToScale = (eeData_63 & 0x000F) + 8;
    KsToScale = (unsigned long)1 << KsToScale;// mod for UNO
    mlx90640->ksTo[0] = eeData_61 & 0x00FF;
    mlx90640->ksTo[1] = (eeData_61 & 0xFF00) >> 8;
    mlx90640->ksTo[2] = eeData_62 & 0x00FF;
    mlx90640->ksTo[3] = (eeData_62 & 0xFF00) >> 8;
    
    for(int i = 0; i < 4; i++)
    {
        if(mlx90640->ksTo[i] > 127)
        {
            mlx90640->ksTo[i] = mlx90640->ksTo[i] -256;
        }
        mlx90640->ksTo[i] = mlx90640->ksTo[i] / KsToScale;
    } 
}

//------------------------------------------------------------------------------

void ExtractAlphaParametersUNO(uint16_t *eeData, int Line, float *alpha_Line)//test12
{
    int accRow[24];
    int accColumn[32];
    int p = 0;
    int alphaRef;
    uint8_t alphaScale;
    uint8_t accRowScale;
    uint8_t accColumnScale;
    uint8_t accRemScale;
    uint16_t eeData_32=pgm_read_word(eeData + 32);

    accRemScale = eeData_32 & 0x000F;
    accColumnScale = (eeData_32 & 0x00F0) >> 4;
    accRowScale = (eeData_32 & 0x0F00) >> 8;
    alphaScale = ((eeData_32 & 0xF000) >> 12) + 30;
    alphaRef = pgm_read_word(eeData + 33);
    
    for(int i = 0; i < 6; i++)
    {
        p = i * 4;
        accRow[p + 0] = (pgm_read_word(eeData + 34 +i) & 0x000F);
        accRow[p + 1] = (pgm_read_word(eeData + 34 +i) & 0x00F0) >> 4;
        accRow[p + 2] = (pgm_read_word(eeData + 34 +i) & 0x0F00) >> 8;
        accRow[p + 3] = (pgm_read_word(eeData + 34 +i) & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 24; i++)
    {
        if (accRow[i] > 7)
        {
            accRow[i] = accRow[i] - 16;
        }
    }
    
    for(int i = 0; i < 8; i++)
    {
        p = i * 4;
        accColumn[p + 0] = (pgm_read_word(eeData + 40 +i) & 0x000F);
        accColumn[p + 1] = (pgm_read_word(eeData + 40 +i) & 0x00F0) >> 4;
        accColumn[p + 2] = (pgm_read_word(eeData + 40 +i) & 0x0F00) >> 8;
        accColumn[p + 3] = (pgm_read_word(eeData + 40 +i) & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 32; i ++)
    {
        if (accColumn[i] > 7)
        {
            accColumn[i] = accColumn[i] - 16;
        }
    }
    
      int i=Line;
      for(int j = 0; j < 32; j ++)
      {
          p = 32 * i +j;
          alpha_Line[j] = (pgm_read_word(eeData + 64 +p) & 0x03F0) >> 4;
          if (alpha_Line[j] > 31)
          {
              alpha_Line[j] = alpha_Line[j] - 64;
          }
          alpha_Line[j] = alpha_Line[j]*(1 << accRemScale);
          alpha_Line[j] = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + alpha_Line[j]);
          alpha_Line[j] = alpha_Line[j] / pow(2,(double)alphaScale);
      }

}


//------------------------------------------------------------------------------

void ExtractOffsetParametersUNO(uint16_t *eeData, int Line, int16_t *offset_Line)
{
    int occRow[24];
    int occColumn[32];
    int p = 0;
    int16_t offsetRef;
    uint8_t occRowScale;
    uint8_t occColumnScale;
    uint8_t occRemScale;
    

    occRemScale = (pgm_read_word(eeData + 16) & 0x000F);
    occColumnScale = (pgm_read_word(eeData + 16) & 0x00F0) >> 4;
    occRowScale = (pgm_read_word(eeData + 16) & 0x0F00) >> 8;
    offsetRef = pgm_read_word(eeData + 17);
    if (offsetRef > 32767)
    {
        offsetRef = offsetRef - 65536;
    }
    
    for(int i = 0; i < 6; i++)
    {
        p = i * 4;
        occRow[p + 0] = (pgm_read_word(eeData + 18 + i) & 0x000F);
        occRow[p + 1] = (pgm_read_word(eeData + 18 + i) & 0x00F0) >> 4;
        occRow[p + 2] = (pgm_read_word(eeData + 18 + i) & 0x0F00) >> 8;
        occRow[p + 3] = (pgm_read_word(eeData + 18 + i) & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 24; i++)
    {
        if (occRow[i] > 7)
        {
            occRow[i] = occRow[i] - 16;
        }
    }
    
    for(int i = 0; i < 8; i++)
    {
        p = i * 4;
        occColumn[p + 0] = (pgm_read_word(eeData + 24 + i) & 0x000F);
        occColumn[p + 1] = (pgm_read_word(eeData + 24 + i) & 0x00F0) >> 4;
        occColumn[p + 2] = (pgm_read_word(eeData + 24 + i) & 0x0F00) >> 8;
        occColumn[p + 3] = (pgm_read_word(eeData + 24 + i) & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 32; i ++)
    {
        if (occColumn[i] > 7)
        {
            occColumn[i] = occColumn[i] - 16;
        }
    }
    
    int i = Line;
      for(int j = 0; j < 32; j ++)
      {
          p = 32 * i +j;
          offset_Line[j] = (pgm_read_word(eeData + 64 + p) & 0xFC00) >> 10;
          if (offset_Line[j] > 31)
          {
              offset_Line[j] = offset_Line[j] - 64;
          }
          offset_Line[j] = offset_Line[j]*(1 << occRemScale);
          offset_Line[j] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + offset_Line[j]);
      }

}


//------------------------------------------------------------------------------


void ExtractKtaPixelParametersUNO(uint16_t *eeData, int Line, float *kta_Line)
{
    int p = 0;
    int8_t KtaRC[4];
    int8_t KtaRoCo;
    int8_t KtaRoCe;
    int8_t KtaReCo;
    int8_t KtaReCe;
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    uint8_t split;
    uint16_t eeData_54=pgm_read_word(eeData + 54);
    uint16_t eeData_55=pgm_read_word(eeData + 55);
    uint16_t eeData_56=pgm_read_word(eeData + 56);

    KtaRoCo = (eeData_54 & 0xFF00) >> 8;
    if (KtaRoCo > 127)
    {
        KtaRoCo = KtaRoCo - 256;
    }
    KtaRC[0] = KtaRoCo;
    
    KtaReCo = (eeData_54 & 0x00FF);
    if (KtaReCo > 127)
    {
        KtaReCo = KtaReCo - 256;
    }
    KtaRC[2] = KtaReCo;
      
    KtaRoCe = (eeData_55 & 0xFF00) >> 8;
    if (KtaRoCe > 127)
    {
        KtaRoCe = KtaRoCe - 256;
    }
    KtaRC[1] = KtaRoCe;
      
    KtaReCe = (eeData_55 & 0x00FF);
    if (KtaReCe > 127)
    {
        KtaReCe = KtaReCe - 256;
    }
    KtaRC[3] = KtaReCe;
  
    ktaScale1 = ((eeData_56 & 0x00F0) >> 4) + 8;
    ktaScale2 = (eeData_56 & 0x000F);

    int i = Line;
      for(int j = 0; j < 32; j ++)
      {
          p = 32 * i +j;
          split = 2*(p/32 - (p/64)*2) + p%2;
          kta_Line[j] = (pgm_read_word(eeData + 64 + p) & 0x000E) >> 1;
          if (kta_Line[j] > 3)
          {
              kta_Line[j] = kta_Line[j] - 8;
          }
          kta_Line[j] = kta_Line[j] * (1 << ktaScale2);
          kta_Line[j] = KtaRC[split] + kta_Line[j];
          kta_Line[j] = kta_Line[j] / pow(2,(double)ktaScale1);
      }
}


//------------------------------------------------------------------------------

void ExtractKvPixelParametersUNO(uint16_t *eeData, int Line, float *kv_Line)
{
    int p = 0;
    int8_t KvT[4];
    int8_t KvRoCo;
    int8_t KvRoCe;
    int8_t KvReCo;
    int8_t KvReCe;
    uint8_t kvScale;
    uint8_t split;
    uint16_t eeData_52=pgm_read_word(eeData + 52);
    uint16_t eeData_56=pgm_read_word(eeData + 56);

    KvRoCo = (eeData_52 & 0xF000) >> 12;
    if (KvRoCo > 7)
    {
        KvRoCo = KvRoCo - 16;
    }
    KvT[0] = KvRoCo;
    
    KvReCo = (eeData_52 & 0x0F00) >> 8;
    if (KvReCo > 7)
    {
        KvReCo = KvReCo - 16;
    }
    KvT[2] = KvReCo;
      
    KvRoCe = (eeData_52 & 0x00F0) >> 4;
    if (KvRoCe > 7)
    {
        KvRoCe = KvRoCe - 16;
    }
    KvT[1] = KvRoCe;
      
    KvReCe = (eeData_52 & 0x000F);
    if (KvReCe > 7)
    {
        KvReCe = KvReCe - 16;
    }
    KvT[3] = KvReCe;
  
    kvScale = (eeData_56 & 0x0F00) >> 8;

    int i = Line;
      for(int j = 0; j < 32; j ++)
      {
          p = 32 * i +j;
          split = 2*(p/32 - (p/64)*2) + p%2;
          kv_Line[j] = KvT[split];
          kv_Line[j] = kv_Line[j] / pow(2,(double)kvScale);
      }
}


//------------------------------------------------------------------------------

void ExtractCPParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    float alphaSP[2];
    int16_t offsetSP[2];
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;
    uint16_t eeData_32=pgm_read_word(eeData + 32);
    uint16_t eeData_58=pgm_read_word(eeData + 58);
    uint16_t eeData_57=pgm_read_word(eeData + 57);
    uint16_t eeData_59=pgm_read_word(eeData + 59);
    uint16_t eeData_56=pgm_read_word(eeData + 56);

    alphaScale = ((eeData_32 & 0xF000) >> 12) + 27;
    
    offsetSP[0] = (eeData_58 & 0x03FF);
    if (offsetSP[0] > 511)
    {
        offsetSP[0] = offsetSP[0] - 1024;
    }
    
    offsetSP[1] = (eeData_58 & 0xFC00) >> 10;
    if (offsetSP[1] > 31)
    {
        offsetSP[1] = offsetSP[1] - 64;
    }
    offsetSP[1] = offsetSP[1] + offsetSP[0]; 
    
    alphaSP[0] = (eeData_57 & 0x03FF);
    if (alphaSP[0] > 511)
    {
        alphaSP[0] = alphaSP[0] - 1024;
    }
    alphaSP[0] = alphaSP[0] /  pow(2,(double)alphaScale);
    
    alphaSP[1] = (eeData_57 & 0xFC00) >> 10;
    if (alphaSP[1] > 31)
    {
        alphaSP[1] = alphaSP[1] - 64;
    }
    alphaSP[1] = (1 + alphaSP[1]/128) * alphaSP[0];
    
    cpKta = (eeData_59 & 0x00FF);
    if (cpKta > 127)
    {
        cpKta = cpKta - 256;
    }
    ktaScale1 = ((eeData_56 & 0x00F0) >> 4) + 8;    
    mlx90640->cpKta = cpKta / pow(2,(double)ktaScale1);
    
    cpKv = (eeData_59 & 0xFF00) >> 8;
    if (cpKv > 127)
    {
        cpKv = cpKv - 256;
    }
    kvScale = (eeData_56 & 0x0F00) >> 8;
    mlx90640->cpKv = cpKv / pow(2,(double)kvScale);
       
    mlx90640->cpAlpha[0] = alphaSP[0];
    mlx90640->cpAlpha[1] = alphaSP[1];
    mlx90640->cpOffset[0] = offsetSP[0];
    mlx90640->cpOffset[1] = offsetSP[1];  
}

//------------------------------------------------------------------------------

void ExtractCILCParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    float ilChessC[3];
    uint8_t calibrationModeEE;
    uint16_t eeData_10=pgm_read_word(eeData + 10);
    uint16_t eeData_53=pgm_read_word(eeData + 53);
    
    calibrationModeEE = (eeData_10 & 0x0800) >> 4;
    calibrationModeEE = calibrationModeEE ^ 0x80;

    ilChessC[0] = (eeData_53 & 0x003F);
    if (ilChessC[0] > 31)
    {
        ilChessC[0] = ilChessC[0] - 64;
    }
    ilChessC[0] = ilChessC[0] / 16.0f;
    
    ilChessC[1] = (eeData_53 & 0x07C0) >> 6;
    if (ilChessC[1] > 15)
    {
        ilChessC[1] = ilChessC[1] - 32;
    }
    ilChessC[1] = ilChessC[1] / 2.0f;
    
    ilChessC[2] = (eeData_53 & 0xF800) >> 11;
    if (ilChessC[2] > 15)
    {
        ilChessC[2] = ilChessC[2] - 32;
    }
    ilChessC[2] = ilChessC[2] / 8.0f;
    
    mlx90640->calibrationModeEE = calibrationModeEE;
    mlx90640->ilChessC[0] = ilChessC[0];
    mlx90640->ilChessC[1] = ilChessC[1];
    mlx90640->ilChessC[2] = ilChessC[2];
}

//------------------------------------------------------------------------------

int ExtractDeviatingPixelsUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;
    uint16_t outlierPixCnt = 0;
    int warn = 0;
    int i;
    //uint16_t eeData_32=pgm_read_word(eeData + 32);
    
    for(pixCnt = 0; pixCnt<5; pixCnt++)
    {
        mlx90640->brokenPixels[pixCnt] = 0xFFFF;
        mlx90640->outlierPixels[pixCnt] = 0xFFFF;
    }
        
    pixCnt = 0;    
    while (pixCnt < 768 && brokenPixCnt < 5 && outlierPixCnt < 5)
    {
        if(pgm_read_word(eeData + pixCnt+64) == 0)
        {
            mlx90640->brokenPixels[brokenPixCnt] = pixCnt;
            brokenPixCnt = brokenPixCnt + 1;
        }    
        else if((pgm_read_word(eeData + pixCnt+64) & 0x0001) != 0)
        {
            mlx90640->outlierPixels[outlierPixCnt] = pixCnt;
            outlierPixCnt = outlierPixCnt + 1;
        }    
        
        pixCnt = pixCnt + 1;
        
    } 
    
    if(brokenPixCnt > 4)  
    {
        warn = -3;
    }         
    else if(outlierPixCnt > 4)  
    {
        warn = -4;
    }
    else if((brokenPixCnt + outlierPixCnt) > 4)  
    {
        warn = -5;
    } 
    else
    {
        for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
        {
            for(i=pixCnt+1; i<brokenPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt],mlx90640->brokenPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }    
            }    
        }
        
        for(pixCnt=0; pixCnt<outlierPixCnt; pixCnt++)
        {
            for(i=pixCnt+1; i<outlierPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->outlierPixels[pixCnt],mlx90640->outlierPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }    
            }    
        } 
        
        for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
        {
            for(i=0; i<outlierPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt],mlx90640->outlierPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }    
            }    
        }    
        
    }    
    
    
    return warn;
       
}


//------------------------------------------------------------------------------

 int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2)
 {
     int pixPosDif;
     
     pixPosDif = pix1 - pix2;
     if(pixPosDif > -34 && pixPosDif < -30)
     {
         return -6;
     } 
     if(pixPosDif > -2 && pixPosDif < 2)
     {
         return -6;
     } 
     if(pixPosDif > 30 && pixPosDif < 34)
     {
         return -6;
     }
     
     return 0;    
 }
 
 //------------------------------------------------------------------------------
 
 int CheckEEPROMValid(uint16_t *eeData)  
 {
     int deviceSelect;
     deviceSelect = pgm_read_word(eeData + 10) & 0x0040;
     if(deviceSelect == 0)
     {
         return 0;
     }
     
     return -7;    
 }        
