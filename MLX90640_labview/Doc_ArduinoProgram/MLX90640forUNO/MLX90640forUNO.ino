

#include <Wire.h>

#include "MLX90640_API_UNO.h"
#include "MLX90640_I2C_Driver.h"
#include <avr/pgmspace.h>

//　　************************* calibration store. first we run program that dumps the data, and then we cut and paste
//          it here in bracekts below*********

//센서 캘리브레이션 데이터 (EEPROM 주소 0x2400 ~ 832 개)를 플래시 메모리 영역에 저장합니다.
const int eeData[] PROGMEM =  {   //factoryCalData[]
//data goes here
177, 18847, 0, 8289, 5, 800, 992, 4629, 595, 391, 1177, 0, 6401, 0, 0, 48691, 
17168, 65460, 257, 257, 257, 61937, 57585, 53216, 65278, 3840, 1, 258, 260, 61956, 57844, 45778, 
35221, 13907, 61148, 15, 4369, 4385, 4369, 65280, 60876, 15, 8738, 9011, 8755, 4386, 61184, 43997, 
5980, 12201, 7509, 41086, 30310, 63954, 24672, 24156, 9315, 128, 6066, 1605, 60416, 38807, 38807, 11003, 
3134, 1296, 5454, 65516, 1198, 2030, 7294, 62638, 3054, 46, 6238, 62542, 78, 64574, 6142, 60638, 
64574, 63566, 4318, 59454, 64526, 62480, 3054, 60350, 64608, 61440, 4032, 58448, 64592, 61536, 1136, 58640, 
5008, 158, 63710, 6992, 3088, 1886, 28, 4128, 3952, 958, 65534, 4046, 3008, 958, 65422, 2158, 
1984, 65502, 63598, 2992, 1952, 64430, 64398, 2864, 3056, 64382, 65374, 3024, 4064, 65502, 62464, 1152, 
2014, 174, 8254, 65422, 2078, 1952, 7246, 62590, 974, 64558, 6190, 63470, 1022, 64592, 6094, 61630, 
944, 62542, 5166, 60430, 65504, 62480, 3040, 60270, 976, 63408, 4960, 61344, 64574, 62480, 2048, 59520, 
3152, 270, 1198, 6128, 3216, 1038, 220, 4320, 2112, 174, 64700, 4160, 2144, 190, 63550, 3328, 
2080, 63678, 158, 2144, 1120, 63616, 62590, 3024, 3154, 63520, 65502, 3072, 2226, 64640, 62590, 2256, 
6080, 2160, 7310, 928, 6206, 2992, 8318, 64686, 5102, 1104, 6238, 63598, 1134, 2000, 6126, 62704, 
3056, 1008, 5358, 61504, 64688, 63584, 3166, 60430, 1072, 64464, 4030, 60496, 63696, 62560, 4096, 59664, 
5920, 1984, 65532, 4848, 7056, 1806, 1006, 4080, 5936, 942, 65470, 4000, 2992, 1838, 64318, 2126, 
3906, 846, 63566, 2960, 2034, 64432, 63422, 2896, 4002, 65296, 65326, 2992, 1090, 65456, 64352, 96, 
2080, 3200, 9280, 992, 2192, 1136, 8320, 64656, 3070, 1040, 6208, 65456, 3024, 1104, 7102, 62592, 
64576, 1008, 5262, 60496, 64576, 65520, 3182, 60448, 1074, 65520, 6032, 60496, 1040, 64578, 4112, 61504, 
1138, 64720, 64670, 2112, 1280, 62656, 63726, 3264, 1072, 64576, 63662, 5088, 1024, 64654, 64510, 2224, 
64640, 64528, 63710, 1152, 146, 63552, 61646, 1136, 3202, 63568, 64526, 2192, 3186, 64656, 62576, 2176, 
4928, 2160, 7264, 912, 1184, 2032, 6270, 63600, 1968, 2000, 7072, 63440, 910, 2912, 4960, 62526, 
896, 896, 7086, 62432, 65456, 65408, 4048, 62368, 50, 65504, 6016, 60432, 96, 64512, 3072, 61504, 
1922, 61616, 61646, 2000, 64736, 62510, 61616, 1152, 65488, 62464, 63486, 2032, 65456, 65438, 61342, 1120, 
65490, 64446, 64494, 1008, 64514, 64448, 60446, 1984, 2146, 62480, 63424, 1104, 1186, 63568, 61504, 1138, 
6098, 4130, 7328, 1984, 3296, 3040, 7280, 63760, 4080, 2976, 4160, 63520, 2000, 2978, 4046, 62606, 
992, 16, 4208, 62448, 1008, 65520, 5024, 62448, 64688, 63618, 4064, 59648, 63712, 64608, 2112, 61570, 
2034, 63552, 60638, 2032, 64800, 62464, 61630, 63808, 50, 63408, 59518, 64592, 65522, 64448, 60414, 144, 
64530, 62512, 60590, 32, 64546, 62496, 62430, 2, 64754, 60576, 61488, 63776, 64802, 63616, 60528, 64688, 
63600, 2226, 4352, 64592, 64816, 1152, 4304, 62688, 65504, 32, 6112, 61536, 64496, 112, 2096, 61632, 
63536, 65504, 3248, 59552, 61584, 65506, 2064, 59488, 63696, 63600, 2080, 60608, 63728, 64610, 162, 59634, 
1954, 1984, 63550, 6000, 2114, 1952, 1022, 6128, 4882, 2880, 1806, 6000, 3856, 2960, 64350, 7120, 
4930, 3856, 1998, 6048, 4002, 3856, 1840, 7040, 6130, 4000, 2896, 8130, 6178, 6016, 1968, 6144, 
60464, 1282, 2256, 62544, 62752, 64, 3216, 61680, 64416, 64544, 3040, 60464, 61440, 960, 1984, 59600, 
60496, 65488, 2096, 59424, 59488, 62498, 992, 60384, 62610, 61602, 80, 59488, 62672, 64594, 64626, 59570, 
63554, 62736, 61648, 2146, 64834, 63584, 62656, 2288, 1986, 64560, 62494, 3136, 32, 2000, 63470, 3296, 
114, 2016, 64606, 3120, 130, 1072, 63488, 5104, 4274, 192, 64624, 6242, 4338, 3168, 63632, 5330, 
65410, 50, 114, 62448, 62576, 64434, 32, 60464, 62448, 64416, 65376, 60400, 63376, 64432, 64384, 58480, 
62368, 63424, 64608, 57392, 61376, 62434, 62480, 59344, 59554, 60546, 64400, 57440, 60498, 62482, 61440, 57488, 
64402, 61488, 57456, 65522, 62594, 62416, 58416, 64578, 62466, 62400, 61328, 63520, 65474, 64448, 60304, 114, 
930, 64448, 59518, 64, 994, 63504, 59440, 3024, 194, 62608, 64416, 80, 2130, 16, 61440, 2210, 
63360, 4114, 3088, 848, 63584, 4914, 4096, 64576, 800, 4898, 4960, 65376, 768, 4914, 4848, 928, 
64384, 4834, 5040, 64352, 64400, 2992, 3904, 720, 2914, 3986, 5922, 65440, 976, 5874, 2994, 992, 
63346, 64496, 58400, 3906, 62546, 65328, 60432, 2128, 1842, 1824, 63358, 5970, 2786, 2832, 64256, 7074, 
1922, 4848, 65518, 6002, 3986, 2992, 65376, 8930, 8050, 3968, 1824, 9104, 7120, 8944, 928, 10226, 
59264, 1074, 64578, 64336, 59424, 898, 2, 61552, 61232, 992, 1984, 63424, 61216, 1856, 1840, 62464, 
60400, 928, 3008, 62416, 59392, 1874, 1904, 61424, 61504, 944, 2880, 64416, 65424, 2962, 2914, 62576, 
57186, 58384, 53296, 64306, 59378, 60224, 55296, 62562, 61234, 61408, 57312, 978, 62288, 64336, 59200, 1040, 
62450, 64400, 61392, 2000, 61474, 65376, 61296, 2018, 50, 912, 63280, 5010, 5010, 3968, 64352, 5218, 
930, 9186, 3122, 3890, 1986, 6946, 4048, 3024, 1888, 6130, 4032, 1024, 976, 8034, 3984, 2112, 
64512, 8082, 7168, 2976, 3056, 9010, 7056, 3904, 1072, 7106, 5888, 3968, 1056, 10114, 3040, 5168, 
61314, 64434, 56336, 2818, 65442, 65264, 59312, 4018, 834, 65472, 61344, 3056, 960, 4928, 62336, 4144, 
18, 3984, 64528, 8114, 5074, 6944, 65424, 10002, 4114, 4000, 1792, 9072, 6162, 9056, 2000, 11280, 
//date ends here
};
//**********************end of factort calibration data************************************************************


const byte i2cAddr = 0x33;
uint16_t frameParams[66];
float T_Line[32];
//uint16_t eeData[832];
paramsMLX90640UNO SnrParamsUNO;

uint16_t response;
const float ta_Shift = 8.0;
const float emissivity = 0.95;

const uint16_t CtrlReg_addr = 0x800D;
//const uint16_t CtrlReg_data = 0b0001101010000001;//16hz
//const uint16_t CtrlReg_data = 0b0001101000000001;//8hz
//const uint16_t CtrlReg_data = 0b0001100110000001;//4hz
//const uint16_t CtrlReg_data = 0b0001100100000001;//2hz
//const uint16_t CtrlReg_data = 0b0001100010000001;//1hz
const uint16_t CtrlReg_data = 0b0001100000000001;//0.5hz
const uint16_t CtrlReg_mask = 0b1110000000000000;

const uint16_t StatusReg_addr = 0x8000;
const uint16_t data_0x8000 = 0b000000000101000;//StartMeasurement,NewData

unsigned long millis();

//int RES;

void setup()
{
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal


  if (isConnected() == false){
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }



  response = MLX90640_ExtractParametersUNO(eeData, &SnrParamsUNO);
  
  //checkExtractedParams(&SnrParamsUNO);
  
  Setup_MLX90640();//1Hz
  
}

void loop()
{
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    unsigned long time_now=millis();
    response = MLX90640_WaitFrameDataUNO(i2cAddr);
    //Serial.println("----------------------------------------------------------------------");
    Serial.println("StartOfImage");
    MLX90640_GetFrameParamsUNO(i2cAddr, frameParams);
    for(int Line=0;Line<24;Line++){
      MLX90640_CalculateToUNO(i2cAddr, frameParams, eeData, &SnrParamsUNO, emissivity, ta_Shift, Line, T_Line);
      for(int j=0;j<32;j++){
        Serial.print(T_Line[j], 1);
        Serial.print(",");
      }
      Serial.println();
    }
    //Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    //Serial.println(millis()-time_now);
    }
}

void Setup_MLX90640() {
  uint16_t CtrlReg_set;
  MLX90640_I2CRead(i2cAddr, CtrlReg_addr, 1, &response);
  delay(5);
  CtrlReg_set=response & CtrlReg_mask;
  CtrlReg_set= CtrlReg_set|CtrlReg_data;
  MLX90640_I2CWrite(i2cAddr, CtrlReg_addr, CtrlReg_set);
  delay(5);
}

boolean isConnected()
{
  Wire.beginTransmission((uint8_t)i2cAddr);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

