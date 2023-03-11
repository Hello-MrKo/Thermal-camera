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
#include <avr/pgmspace.h>
#include <arduino.h>

#ifndef _MLX640_API_UNO_H_
#define _MLX640_API_UNO_H_

  typedef struct
    {
        int16_t kVdd;
        int16_t vdd25;
        float KvPTAT;
        float KtPTAT;
        uint16_t vPTAT25;
        float alphaPTAT;
        int16_t gainEE;
        float tgc;
        float cpKv;
        float cpKta;
        uint8_t resolutionEE;
        uint8_t calibrationModeEE;
        float KsTa;
        float ksTo[4];
        int16_t ct[4];
        float cpAlpha[2];
        int16_t cpOffset[2];
        float ilChessC[3]; 
        uint16_t brokenPixels[5];
        uint16_t outlierPixels[5];  
    } paramsMLX90640UNO;

    uint16_t test_eeData(uint16_t *eeData, int num);
    int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t *eeData);

    int MLX90640_ExtractParametersUNO(uint16_t *eeData, paramsMLX90640UNO *mlx90640);
    void ExtractAlphaParametersUNO(uint16_t *eeData, int Line, float *alpha_Line);
    void ExtractOffsetParametersUNO(uint16_t *eeData, int Line, int16_t *offset_Line);
    void ExtractKtaPixelParametersUNO(uint16_t *eeData, int Line, float *offset_Line);
    void ExtractKvPixelParametersUNO(uint16_t *eeData, int Line, float *kv_Line);
    void MLX90640_CalculateToUNO(uint8_t slaveAddr, uint16_t *frameParams, uint16_t *eeData, paramsMLX90640UNO *SnrParamsUNO, float emissivity, float ta_Shift, int Line, float *T_Line);
    int MLX90640_GetFrameParamsUNO(uint8_t slaveAddr, uint16_t *frameParams);
    int MLX90640_WaitFrameDataUNO(uint8_t slaveAddr);
    
#endif

