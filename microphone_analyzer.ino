#include "Arduino.h";
#include "arduinoFFT.h"
#include "SoundData.h"
#include "XT_DAC_Audio.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

#define SAMPLES 256             //Must be a power of 2
#define SAMPLING_FREQUENCY 22000 //Hz



TaskHandle_t samplingHandle;

/*WIFI Stuff*/

const char* ssid     = "Ramy's iPhone";
const char* password = "ramy1234";



// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

XT_Wav_Class sineSweep(sweep);

XT_Wav_Class oneKSignal(oneK);
XT_Wav_Class threeKSignal(threeK);
XT_Wav_Class twoKSignal(twoK);
// create an object of type XT_Wav_Class that is used by
// the dac audio class (below), passing wav data as parameter.

XT_DAC_Audio_Class DacAudio(25, 0);   // Create the main player class object.
// Use GPIO 25, one of the 2 DAC pins and timer 0


arduinoFFT FFT = arduinoFFT();

unsigned long samplingStartTime;
const int micPin = 33;
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
int samplingRounds = 0;
int freqCounter = 0;
int angle1kCounter = 0;
int angle3kCounter = 0;
int angle2kCounter = 0;

double avgAmp [7];
bool freqPlayed = false;    //play it only once
bool freqStart = false;     //no plotting on startup
bool freqResponse = false;  //SIGNAL


double tempAvg[3];
double avg1k [3];
double avg3k [3];
double avg2k [3];
bool directivity = false;

bool firstAngle = false;
bool secondAngle = false;
bool thirdAngle = false;

bool oneKhzPlayed = false;
bool oneKhzStart = false;
bool oneKhz = false;

bool threeKhzPlayed = false;
bool threeKhzStart = false;
bool threeKhz = false;

bool twoKhzPlayed = false;
bool twoKhzStart = false;
bool twoKhz = false;

bool sensitivity = false;
int sensitivityValue;
float vRMS;
int gainFactor = 160;
int adcValue;
float voltage;
float offset;
int offsetCounter;
int c;

/*SAMPLING*/
void sampling(void * parameters) {
  for (;;) {
    vTaskDelay(1);

    //------------------------------------SENSITIVITY--------------------------------------//

    if (sensitivity) {
      delay(1000);
      samplingStartTime = millis();
      while (millis() - samplingStartTime < 250) { //calculating offset
        adcValue = analogRead(micPin);
        voltage = (float) adcValue * (1.0 / 4095);
        offset += voltage;
        offsetCounter++;
      }
      offset /= offsetCounter;

      while (millis() - samplingStartTime < 750) {
        adcValue = analogRead(micPin);
        voltage = (float) adcValue * (1.0 / 4095);
        vRMS += pow((voltage - offset), 2);
        c++;
      }
      //-------------------------------------- SAMPLING DONE --------------------------------------

      vRMS /= c;
      sensitivityValue = (20 * log10(vRMS / gainFactor)) + 16;  //16 is added due to measuring at ~78dB


      oneKhz = false;
      oneKhzPlayed = false;
      sensitivity = false;
      offset = 0.0;
      c = 0;
      offsetCounter = 0;
      vRMS = 0.0;
    }


    //------------------------------------DIRECTIVITY--------------------------------------//

    if (directivity) {
      delay(1000);
      for (int i = 0; i < 3; i++)
      {
        tempAvg[i] = 0;
      }
      samplingStartTime = millis();
      while (millis() - samplingStartTime < 1250) {

        for (int i = 0; i < SAMPLES; i++)
        {
          microseconds = micros();    //Overflows after around 70 minutes!

          vReal[i] = analogRead(micPin);
          vImag[i] = 0;

          while (micros() < (microseconds + sampling_period_us)) {
          }
        }

        /*FFT*/
        FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
        FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
        double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);


        tempAvg[0] += vReal[12];  // 1031 Hz
        tempAvg[1] += vReal[36];  // 3093 Hz
        tempAvg[2] += vReal[24];  // 2062 Hz

        samplingRounds++;
      }
      //-------------------------------------- SAMPLING DONE --------------------------------------


      for (int i = 0; i < 3; i++)
      {
        tempAvg[i] /= samplingRounds;
      }

      if (oneKhz) {
        oneKhzStart = true;
        if (firstAngle) {
          avg1k[0] = tempAvg[0];
          firstAngle = false;
        }
        else if (secondAngle) {
          avg1k[1] = tempAvg[0];
          secondAngle = false;
        }
        else if (thirdAngle) {
          avg1k[2] = tempAvg[0];
          thirdAngle = false;
        }
        oneKhz = false;
        oneKhzPlayed = false;
        angle1kCounter = 0;
      }

      if (threeKhz) {
        threeKhzStart = true;
        if (firstAngle) {
          avg3k[0] = tempAvg[1];
          firstAngle = false;
        }
        else if (secondAngle) {
          avg3k[1] = tempAvg[1];
          secondAngle = false;
        }
        else if (thirdAngle) {
          avg3k[2] = tempAvg[1];
          thirdAngle = false;
        }
        threeKhz = false;
        threeKhzPlayed = false;
        angle3kCounter = 0;
      }

      if (twoKhz) {
        twoKhzStart = true;
        if (firstAngle) {
          avg2k[0] = tempAvg[2];
          firstAngle = false;
        }
        else if (secondAngle) {
          avg2k[1] = tempAvg[2];
          secondAngle = false;
        }
        else if (thirdAngle) {
          avg2k[2] = tempAvg[2];
          thirdAngle = false;
        }
        twoKhz = false;
        twoKhzPlayed = false;
        angle2kCounter = 0;
      }

      samplingRounds = 0;
      directivity = false;
    }

    //------------------------------------FREQ RESPONSE--------------------------------------//

    if (freqResponse) {
      delay(1000);
      freqStart = true;
      for (int i = 0; i < 7; i++)
      {
        avgAmp[i] = 0;
      }
      samplingStartTime = millis();
      while (millis() - samplingStartTime < 2100) {

        for (int i = 0; i < SAMPLES; i++)
        {
          microseconds = micros();    //Overflows after around 70 minutes!

          vReal[i] = analogRead(micPin);
          vImag[i] = 0;

          while (micros() < (microseconds + sampling_period_us)) {
          }
        }

        /*FFT*/
        FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
        FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
        double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);


        avgAmp[0] += vReal[2];  // 171 Hz
        avgAmp[1] += vReal[4];  // 343 Hz
        avgAmp[2] += vReal[7];  // 601 Hz
        avgAmp[3] += vReal[14]; // 1203 Hz
        avgAmp[4] += vReal[30]; // 2578 Hz
        avgAmp[5] += vReal[59]; // 5070 Hz
        avgAmp[6] += vReal[121];// 10.4k Hz

        samplingRounds++;
      }
      //-------------------------------------- SAMPLING DONE --------------------------------------


      for (int i = 0; i < 7; i++)
      {
        avgAmp[i] /= samplingRounds;

      }

      ///// TEST DONE /////
      freqResponse = false;
      freqPlayed = false;
      samplingRounds = 0;
      freqCounter = 0;

    }

  }
}

String processor(const String& var) {
  if (var == "SENSITIVITY") {
    return String(sensitivityValue);
  }
  return String();
}

String getAngle1k() {
  double temp1k = avg1k[angle1kCounter];
  angle1kCounter++;
  return String(temp1k);
}

String getAngle3k() {
  double temp3k = avg3k[angle3kCounter];
  angle3kCounter++;
  return String(temp3k);
}

String getAngle2k() {
  double temp2k = avg2k[angle2kCounter];
  angle2kCounter++;
  return String(temp2k);
}

String getAmp() {
  double amplitude = avgAmp[freqCounter];
  freqCounter++;
  return String(amplitude);
}

void setup() {
  Serial.begin(115200);

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  pinMode(micPin, INPUT);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));


  xTaskCreatePinnedToCore(
    sampling,   /* Task function. */
    "sampling",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &samplingHandle,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */



  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });



  //-------------------------Signal Engine-------------------------//
  server.on("/sensitivity", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!sensitivity && !freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("Sensitivity test has been initiated");
      sensitivity = true;
      oneKhz = true;
    }
    delay(2500);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/resetSensitivity", HTTP_GET, [](AsyncWebServerRequest * request) {
    sensitivityValue = 0;
    delay(1000);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/freqResponse", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("Frequency Response test has been initiated");
      freqResponse = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  //1k
  server.on("/first1kTest", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("1K Hz signal will start playing");
      directivity = true;
      oneKhz = true;
      firstAngle = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/second1kTest", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("1K Hz signal will start playing");
      directivity = true;
      oneKhz = true;
      secondAngle = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/third1kTest", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("1K Hz signal will start playing");
      directivity = true;
      oneKhz = true;
      thirdAngle = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  //3k
  server.on("/first3kTest", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("3K Hz signal will start playing");
      directivity = true;
      threeKhz = true;
      firstAngle = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/second3kTest", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("3K Hz signal will start playing");
      directivity = true;
      threeKhz = true;
      secondAngle = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/third3kTest", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("3K Hz signal will start playing");
      directivity = true;
      threeKhz = true;
      thirdAngle = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  //2k
  server.on("/first2kTest", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("2k Hz signal will start playing");
      directivity = true;
      twoKhz = true;
      firstAngle = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/second2kTest", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("2k Hz signal will start playing");
      directivity = true;
      twoKhz = true;
      secondAngle = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/third2kTest", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("2k Hz signal will start playing");
      directivity = true;
      twoKhz = true;
      thirdAngle = true;
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/reset1k", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("1k results will now reset");
      for (int i = 0; i < 3; i++) {
        avg1k[i] = 0;
      }
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/reset3k", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("3k results will now reset");
      for (int i = 0; i < 3; i++) {
        avg3k[i] = 0;
      }
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/reset2k", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!freqResponse && !oneKhz && !threeKhz && !twoKhz) {
      Serial.println("2k results will now reset");
      for (int i = 0; i < 3; i++) {
        avg2k[i] = 0;
      }
    }
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });


  //-------------------------PLOTTING-------------------------//

  server.on("/plotAmp", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (freqStart && !freqResponse && freqCounter < 7) {
      String amp = getAmp().c_str();
      request->send_P(200, "text/plain", amp.c_str());
    }
  });

  server.on("/plot1k", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (oneKhzStart && !oneKhz && angle1kCounter < 3) {
      String oneKAngle = getAngle1k().c_str();
      request->send_P(200, "text/plain", oneKAngle.c_str());
    }
  });

  server.on("/plot3k", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (threeKhzStart && !threeKhz && angle3kCounter < 3) {
      String threeKAngle = getAngle3k().c_str();
      request->send_P(200, "text/plain", threeKAngle.c_str());
    }
  });

  server.on("/plot2k", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (twoKhzStart && !twoKhz && angle2kCounter < 3) {
      String twoKAngle = getAngle2k().c_str();
      request->send_P(200, "text/plain", twoKAngle.c_str());
    }
  });

  server.begin();
}

void loop() {
  DacAudio.FillBuffer();                // Fill the sound buffer with data

  if (sineSweep.Playing == false && !freqPlayed && freqResponse) {
    delay(1000);
    DacAudio.Play(&sineSweep);
    freqPlayed = true;
  }

  if (oneKSignal.Playing == false && !oneKhzPlayed && oneKhz) {
    delay(1000);
    DacAudio.Play(&oneKSignal);
    oneKhzPlayed = true;
  }

  if (threeKSignal.Playing == false && !threeKhzPlayed && threeKhz) {
    delay(1000);
    DacAudio.Play(&threeKSignal);
    threeKhzPlayed = true;
  }

  if (twoKSignal.Playing == false && !twoKhzPlayed && twoKhz) {
    delay(1000);
    DacAudio.Play(&twoKSignal);
    twoKhzPlayed = true;
  }

}
