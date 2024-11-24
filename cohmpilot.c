// Ohmpilot controller
// license: MIT License 

//install dev: apt-get install libcurl4-openssl-dev libmodbus-dev
//compile: gcc -o cohmpilot cohmpilot.c -I/usr/include/modbus -lmodbus -lcurl -Wall


#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <modbus.h>
#include <curl/curl.h>

//config
//const int cMaxHardware = 52; // Actual heater physical limit setting
const int cMaxtemperature = 52.5;
// PT1000 to DS1820 correction, verified via analog temperture sensor 
const float cTempK = 1.03;
const float cTempD = 3.80;
const int cTimeIntervalHours = 6;
const int cMinimalHeaterPower = 275; //Watt
const int cReservedPower = 105;      //Watt
const int cMinSetPowerChange = 15;   //Watt
const int cSleepSec = 20;  // Update interval, 1-45 sec possible, 

//SHRDZM device
char cSmartMeterIP[] = "192.168.0.11";
char cUSER[] = "4E2609D53341";
char cPASS[] = "18321456";
char cCSV[] = "~/ohmpilotdata";

//Modbus addresses Ohmpilot
const char MBServer[]  = "192.168.1.101";


static sig_atomic_t endProg = 0;

static void sighandler(int signo) {
  endProg = 1;
}

struct PowerData {
  int inpower_1_7_0;  // Aktuelle Leistung +P - Bezug 
  int outpower_2_7_0; // Aktuelle Leistung -P - Lieferung 
};

static size_t AnswerFunc(void *contents, size_t size, size_t nmemb, void *userp) {
  struct PowerData *powerdata = (struct PowerData *)userp;

  int scan1 = 0, scan2 = 0;
  powerdata->inpower_1_7_0 = 0;
  powerdata->outpower_2_7_0 = 0;
  char* pValue = strstr(contents,"\"1.7.0\":\"");
  if (pValue) {
    scan1 = sscanf(&pValue[9], "%d", &powerdata->inpower_1_7_0);
  }
  pValue = strstr(contents,"\"2.7.0\":\"");
  if (pValue) {
    scan2 = sscanf(&pValue[9], "%d", &powerdata->outpower_2_7_0);
  } 
  if (scan1>0 && scan2>0) {
    return nmemb;
  }
  return 0;
}



int GetPowerSmartMeter(CURL* curl, struct PowerData* powerdata) {
  powerdata->inpower_1_7_0 = 0;
  powerdata->outpower_2_7_0 = 0;

  if (curl) {
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
      fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
    } else {
      printf("   out power: %4d Watt\n", powerdata->outpower_2_7_0);
      printf("    in power: %4d Watt\n", powerdata->inpower_1_7_0);
    }
    return res;
  }
  return -1;
}

const int  AddManuf  = 40004; // Manufacturer
const int  AddDevice = 40009; // Device Name
const int  AddSN     = 40023; // SN
const int  AddSWVer  = 40041; // SW version

const int  AddrMaxPowerDevice = 40405; // Max Power device
const int  AddrMaxPowerHeater = 40408; // Max Power heater

const int  AddSetTime  = 40399; // 2reg:0000? 2reg:time 1reg:00?  
const int  AddSetPower = 40599;

const int  AddStatus   = 40799;
const int  AddrActPower= 40800; //Watt
const int  AddrEnergy  = 40804; //Wh
const int  AddrTemp    = 40808; // 0.1 *C 

//Modbus register access and convert  
void string_copy(char *des, const uint16_t* registers, int length) {
  des[2*length-1] = '\0';
  des[2*length] = '\0';
  const char *data = (const char *) registers;
  for (int i = 0; i < length; i++) {
      int pos = i*2; 
      des[pos] = data[pos+1];
      des[pos+1] = data[pos];
  }
}

void int16_copy(uint16_t *des, const uint16_t* registers) {
  des[0] = registers[0];
}

void int32_copy(uint32_t *des, const uint16_t* registers) {
  uint16_t *des_int16 = (uint16_t *)des;
  des_int16[0] = registers[1];
  des_int16[1] = registers[0];
}

void int64_copy(uint64_t* des, const uint16_t *value) {
  uint16_t *des_int16 = (uint16_t *)des;
  des_int16[3] = value[1];
  des_int16[2] = value[0];
  des_int16[0] = value[4];
  des_int16[1] = value[3]; 
}

void int32_copy_write(uint16_t* des, const uint32_t *value) {
  uint16_t *value_int16 = (uint16_t *)value;
  des[0] = value_int16[1];
  des[1] = value_int16[0];
}


void zeroRegisters(uint16_t* regbuffer, int size) {
  memset(regbuffer, 0, sizeof(uint16_t)*size);
}


int reportModbusError(const int result) {
  printf("failed\n");
  if (result!=0) {
   fprintf(stderr, "modbus operation failed, result:%d, error:%s\n", result, modbus_strerror(errno));
   return errno;
  }
  return 0;
 }


int readRegisters(uint16_t* inbuffer, const uint16_t Addr, const uint16_t size, modbus_t *mb) {
  int result = 0;
  zeroRegisters(inbuffer, size);
  printf("<< read registes %d-%d address ... ", Addr, Addr+size-1);
  result = modbus_read_registers(mb, Addr, size, inbuffer);
  if (size==result) {
    printf("success\n");
    return 1;
  } else {
    reportModbusError(result);
    return 0;
  }
} 

void setPower(uint32_t  setvalue, modbus_t *mb) {
  uint16_t outbuffer[4];
  int RegSize = 2;

  int32_copy_write(outbuffer, &setvalue);
  printf(">> write register %d address ... ", AddSetPower);
  int result = modbus_write_registers(mb, AddSetPower, RegSize, outbuffer);
  if (RegSize==result) {
    printf("success\n");
  } else {
    printf("failed\n");
  }
}

void getTime(modbus_t *mb) {
  uint16_t inbuffer[6];
  time_t optime = 0;
  uint16_t optime2 = 0;

  if (readRegisters(inbuffer, AddSetTime, 5, mb)) {
    int32_copy((uint32_t*)&optime, &inbuffer[2]);
    int16_copy((uint16_t*)&optime2, &inbuffer[4]);
    printf("Time: %ld, %d - %s\n", optime, optime2, asctime(gmtime(&optime)));
  }
}

void setTime(modbus_t *mb) {
  uint16_t outbuffer[5];
  int RegSize = 5;
  uint32_t  setvalue;

  time_t current_time;
  time(&current_time);
  //printf("UTC: %lx\n", current_time);
  struct tm local_time = *localtime(&current_time);
  //printf("Local time: %02d:%02d:%02d\n", local_time.tm_hour, local_time.tm_min, local_time.tm_sec);
  time_t local_timestamp = timegm(&local_time);
  //printf("CET: %lx\n", local_timestamp);
  setvalue = local_timestamp;

  zeroRegisters(outbuffer, RegSize);
  int32_copy_write(&outbuffer[2], &setvalue);
  printf(">> write register %d address ... ", AddSetTime);
  int result = modbus_write_registers(mb, AddSetTime, RegSize, outbuffer);
  if (RegSize==result) {
    printf("success\n");
  } else {
    printf("failed\n");
  }
}



void write_to_csv_text(const char* filename, const char* data) {
  FILE *file = fopen(filename, "a+");
  if (file == NULL) {
      perror("Error opening csv file for write text");
      return;
  }
  fprintf(file, "%s\n", data);
  fclose(file);
}

void write_to_csv(const char* filename, const float tempCorr, const struct PowerData* power, const uint16_t ActPower, const uint16_t SetPower) {
    char timebuffer[256];
    time_t now;
    struct tm *timeinfo;
    time(&now);
    timeinfo = localtime(&now);
    strftime(timebuffer, sizeof(timebuffer), "%d.%m.%Y\t%H:%M:%S", timeinfo);

    FILE *file = fopen(filename, "a");
    if (file == NULL) {
        perror("Error opening csv file");
        return;
    }
    fprintf(file, "%s\t%.1f\t%hu\t%hu\t%hu\t%hu\n", timebuffer, tempCorr, power->inpower_1_7_0, power->outpower_2_7_0, ActPower, SetPower);
    fclose(file);
}

void csv_filename(char* filename) {
    time_t now;
    struct tm *timeinfo;
    time(&now);
    timeinfo = localtime(&now);

    sprintf(filename, "%s_%04d-%02d-%02d.csv", cCSV, timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday);
    printf("CSV file: %s\n", filename);
    write_to_csv_text(filename, "Date\tTime\tTemp\tIn Power\tOut Power\tHeater act Power\tHeater set Power");
}


int main(void) {
  char URL[512];
  char csvfile[256];

  csv_filename(csvfile);

	struct sigaction sa;
	memset(&sa, 0, sizeof(struct sigaction));
	sa.sa_handler = sighandler;
	sigaction(SIGINT,  &sa, NULL);
	sigaction(SIGQUIT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

  curl_global_init(CURL_GLOBAL_ALL);
  CURL* curl = curl_easy_init();

  struct curl_slist *chunk = NULL;
  CURLcode CurlResult;

  chunk = curl_slist_append(chunk, "Content-Type: application/json");
  CurlResult = curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
  if (CURLE_OK!= CurlResult) {
    fprintf(stderr, "curl_easy_setopt(CURLOPT_HTTPHEADER) failed: %s\n", curl_easy_strerror(CurlResult));
  }
  sprintf(URL, "http://%s/getLastData?user=%s&password=%s", cSmartMeterIP, cUSER, cPASS);
  printf("URL: %s\n", URL);
  CurlResult = curl_easy_setopt(curl, CURLOPT_URL, URL);
  if (CURLE_OK!= CurlResult) {
    fprintf(stderr, "curl_easy_setopt(CURLOPT_URL) failed: %s\n", curl_easy_strerror(CurlResult));
  }

  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, AnswerFunc);
  struct PowerData powerdata;
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&powerdata);

  modbus_t *mb;
  uint16_t inbuffer[100];
  uint16_t status = 0;
  uint16_t temp = 0;
  float tempCorr = 0.0;
  uint32_t actpower = 0;
  uint64_t energy = 0;
  uint32_t outpower = 0;
  //uint32_t maxpower = 0;
  int SetValueOld = 0;
  
  char Manufacturer[22];
  char Device[22];
  char SN[22];
  struct sVersion {
    uint8_t subbuild;
    uint8_t patch;
    uint8_t minor;
    uint8_t major;
  } version;
  char SWversion[20];
  int result;

  printf("connect to %s ... ", MBServer);
  mb = modbus_new_tcp(MBServer, 502);
  //modbus_set_debug(mb, 1);   // Active debug
  modbus_set_slave(mb ,0x28);
  modbus_set_response_timeout(mb, 2, 0);
  result = modbus_connect(mb);
  if (result!=0) {
   reportModbusError(result);
   fprintf(stderr, "Modbus could not connect to %s:502\n", MBServer);
   return errno;
  }
  printf("success\n");

  if (readRegisters(inbuffer, AddManuf, 39, mb)) {
    string_copy(Manufacturer, &inbuffer[0], 5);
    string_copy(Device, &inbuffer[AddDevice-AddManuf], 10);
    printf("Device: %s %s\n", Manufacturer, Device);
    string_copy(SN, &inbuffer[AddSN-AddManuf], 8);
    printf("Serial number: %s\n", SN);
    int32_copy((uint32_t*)&version, &inbuffer[AddSWVer-AddManuf]);
    sprintf(SWversion, "%d.%d.%d-%d", version.major, version.minor, version.patch, version.subbuild);
    printf("Version: %s\n", SWversion);
  }

  const int loopTimeSet = (cTimeIntervalHours*60*60)*(60/cSleepSec);
  printf("Time cycle %u\n", loopTimeSet);
  unsigned int loopCounter = 0;
  while(!endProg) {
    printf("Cycle %u\n", loopCounter);
    if (0==(loopCounter%loopTimeSet)) {
     printf("---------------------\n");
     printf("SET TIME\n");
      getTime(mb);
      setTime(mb);
      getTime(mb);
      printf("----------------------\n");
    }
    loopCounter++;

    if (readRegisters(inbuffer, AddStatus, 10, mb)) {
      int16_copy(&status, &inbuffer[0]);
      printf("Status: %u (0x%08X)\n", status, status);
      
      int32_copy(&actpower, &inbuffer[AddrActPower-AddStatus]);
      printf("Actual power: %u Watt\n", actpower);
      
      int64_copy(&energy, &inbuffer[AddrEnergy-AddStatus]);
      printf("Energie: %lld (0x%08llX) Wh\n", energy, energy);

      int16_copy(&temp, &inbuffer[AddrTemp-AddStatus]);
      //float fTemp = (float)(temp) / 10.0f;  // 3.8 too low, correcting
       tempCorr = (float)(temp) / 10.0f * cTempK + cTempD;
      printf("Temperature: %.1f *C (uncorrected %d) *C\n", tempCorr, temp);
    } else {
      setPower(0, mb); // heater off
      printf("Error - reconnecting...\n");
      modbus_close(mb);
      result = modbus_connect(mb);
      if (result!=0) {
        reportModbusError(result);
        fprintf(stderr, "Modbus could not connect to %s:502\n", MBServer);
      }
      continue;
    }

    int SetValue = 0;
    if (CURLE_OK==GetPowerSmartMeter(curl, &powerdata)) {
      int UnusedPower = (powerdata.outpower_2_7_0-powerdata.inpower_1_7_0);
      printf("Unused power: %4d Watt\n", UnusedPower);
      if (tempCorr<cMaxtemperature) {

        SetValue = MAX(0, actpower + UnusedPower - cReservedPower);  // x Watt save, 
        if (actpower<cMinimalHeaterPower) { 
          // off -> on 
          printf("Set power (inactive): %4d Watt\n", SetValue);
        } else {
          // on adjust
          //SetValue = MAX(0, actpower + UnusedPower- cReservedPower); 
          printf("Set power (active): %4d Watt\n", SetValue);
        }
        
        if (SetValue<cMinimalHeaterPower) {
          printf("power below minimal of %d Watt\n", cMinimalHeaterPower);
          SetValue = 0;
        } else {
            if (SetValueOld>=cMinimalHeaterPower && abs(SetValueOld-SetValue)<cMinSetPowerChange) {
              SetValue = SetValueOld;
              printf("Minimum power change using old set power %d Watt\n", SetValue);
            }
        }
      } 
    }

    SetValueOld = SetValue;
    printf("---------------------\n");
    printf("SET POWER TO %d WATT\n", SetValue);
    printf("----------------------\n");
    setPower(SetValue, mb);  //allways set power, need 50 sec interval set value

    if (readRegisters(inbuffer, AddSetPower, 2, mb)) {
      int32_copy(&outpower, inbuffer);
      printf("Heater set power: %u W\n", outpower);
    }

    write_to_csv(csvfile, tempCorr, &powerdata, actpower, outpower);
    sleep(cSleepSec);
  }
  setPower(0, mb); // heater off

  printf(">> close connection\n");
  modbus_close(mb);
  modbus_free(mb);

  if (curl) {
      curl_easy_cleanup(curl);
  }
  curl_global_cleanup();
}

