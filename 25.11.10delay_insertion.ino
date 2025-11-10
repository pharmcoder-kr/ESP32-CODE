// ===== Part 1: 선언부 및 FreeRTOS 태스크 구현 =====

#include <WiFi.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <queue>
#include <TMCStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
struct Job;

#define DIR_PIN      33    // 스텝모터 DIR
#define STEP_PIN     25    // 스텝모터 STEP
#define EN_PIN       13    // 스텝모터 ENABLE (LOW = ON)
#define UART_TX      17    // 스텝모터 UART TX
#define LIMIT_PIN    34    // 홈 센서(EE-SX674) 출력 핀 (외부 6.8kΩ로 3.3V 풀업)
#define R_SENSE      0.11f // 전류 감지 저항
#define DRIVER_ADDR  0b00  // UART 주소
#define SENSOR_D0_PIN  23  // A0/D0 센서 → ESP32 GPIO23
#define HOME_LIGHT_ON false  // L을 +5V에 묶어 Light-ON이면 true로 바꾸세요.
#define STEPS_PER_CM 1000
// ── Water sensor & Buzzer ─────────────────────────────────
#define WATER_SENSOR_PIN          4     // 수위센서 OUT 핀
#define WATER_USE_INTERNAL_PULLUP 0     // 1: INPUT_PULLUP, 0: INPUT
#define WATER_ACTIVE_HIGH         1     // 1: 센싱=HIGH, 0: 센싱=LOW(반전)

#define BUZZER_PIN                22    // 부저 +핀(능동 부저 가정)
#define DEBUG_WATER 1   // 0이면 디버그 출력 끔

// 부저 패턴: 1s ON / 0.5s OFF × 3
#define BUZZ_ON_MS    1000
#define BUZZ_OFF_MS    500
#define BUZZ_REPEAT       3
//#define WATER_USE_INTERNAL_PULLUP 1    // ★ 내부 풀업 사용
// 프로토타입
inline void buzzerOn();
inline void buzzerOff();
bool waterSensed();
void startBuzzerAlertPattern();
void buzzerAlertTask(void* pvParameters);
bool g_lowWaterAlertActive = false;   // HMI 경고 중복 전송 방지용
void showLowWaterAlert();             // 프로토타입


// === Jog speed tuning (half period, us) ===
// 값이 클수록 느리게 회전합니다.
#define JOG_US_START    800   // 시작시 천천히
#define JOG_US_TARGET   400   // 가속 후 목표 속도
#define JOG_ACCEL_PER_STEP 2  // 매 스텝당 half-period 감소량(가속율). 값이 클수록 빨리 가속

// 연속 펄스용 (가변 half-period)
inline void stepPulseJogVariable(uint16_t us_half_period) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(us_half_period);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(us_half_period);
}

// 조그 상태: 0=정지, 1=정방향(병쪽), 2=역방향(홈쪽)
volatile uint8_t g_jogDir = 0;

TaskHandle_t jogTaskHandle = nullptr;

// 프로토타입
void jogTask(void* pvParameters);

inline bool homeTriggered() {
  int v = digitalRead(LIMIT_PIN);
  return HOME_LIGHT_ON ? (v == HIGH)  // Light-ON: 빛이 있을 때 싱크 -> LOW, 막히면 HIGH인 제품류
                       : (v == LOW);  // Dark-ON: 막히면 LOW (권장 기본)
}

HardwareSerial  TMCserial(2);
TMC2209Stepper  driver(&TMCserial, R_SENSE, DRIVER_ADDR);

#define NEXTION_RX 18
#define NEXTION_TX 19
HardwareSerial nextion(1);
WiFiServer server(80);

// EEPROM addresses
#define ADDR_SSID      0
#define ADDR_PASSWORD 32
#define ADDR_FLAG     100
#define VALID_FLAG    0xA5
#define ADDR_VOLUME   128
#define ADDR_MARGIN   132
#define ADDR_RATE      136
#define ADDR_RATE100   140
#define ADDR_RATE60    144
#define ADDR_RATE30    148
#define ADDR_STEPSPG     168   // 4바이트: pG가 갈 위치 (pM으로 저장)
#define CAL_SEC_30   2.0f
#define CAL_SEC_60   3.0f
#define CAL_SEC_100  5.0f
#define ADDR_STEPS_FLAG   152
#define STEPS_VALID_FLAG  0xA7
#define ADDR_STEPS30      156   // 4바이트
#define ADDR_STEPS60      160   // 4바이트
#define ADDR_STEPS100     164   // 4바이트

enum DispenseState {
  DSP_IDLE, DSP_HOMING, DSP_HOMED_WAIT, DSP_MOVE, DSP_MOVE_WAIT,
  DSP_PUMP, DSP_PUMP_WAIT, DSP_RETURN, DSP_RETURN_WAIT,
  DSP_COMPLETE, DSP_WAIT_CONFIRM
};

DispenseState dspState = DSP_WAIT_CONFIRM;
unsigned long dspTimer = 0;
uint32_t dspPumpDuration = 0;
// ── 전역 페이지 상태 ─────────────────────────────────────────
String g_currentPage = "";      // 현재 Nextion 페이지 이름

// (이 함수를 다른 곳에서 호출하므로 미리 프로토타입 선언)
void updateCompleteNextLabel();
// 전역에 추가
bool isProcessing = true;
// ── 전역 플래그 ───────────────────────────────────────────────────────────
bool pageSwitchedToProcess = false;  // 이 줄을 추가하세요
// 전역 변수 추가
volatile bool isDispenseReady = false;

String ssidList[6];
String selectedSSID;
String wifiPassword;
String inputBuffer;
bool readyToConnect = false;
int ffCount = 0;

int volumeFlag = 0;
int marginFlag = 0;
int U_volume = 0;
int S_offset = 0;
float rate_mL_per_sec = 1.0f;
int rateFlag = 0;
int rate100Flag = 0;
int rate60Flag = 0;
int rate30Flag = 0;

enum State { IDLE, FIXED_PUMP };
State currentState = IDLE;

uint32_t g_steps30  = 5140;  // 30 mL 초기값
uint32_t g_steps60  = 3470;  // 60 mL 초기값
uint32_t g_steps100 = 1505;  // 100 mL 초기값
// 전역 위치값 (기본값은 기존 6000 스텝 가정)
uint32_t g_stepsPg = 6000;

const uint16_t STEP_US = 50;
// 홈 기준 절대 스텝(조그/이동 시 갱신, homing 시 0으로 리셋)
volatile int32_t g_absSteps = 0;

#define PUMP_EN    26
#define PUMP_PWM   27

struct Job {
  int volume;
  int margin;
  String patient_name;
  bool isUrgent;  // 긴급 작업 플래그 추가
};
std::queue<Job> jobQueue;

SemaphoreHandle_t jobQueueMutex;
TaskHandle_t dispenseTaskHandle;
TaskHandle_t httpTaskHandle;
TaskHandle_t hmiTaskHandle;

// ── Water sensor & Buzzer helpers ─────────────────────────
inline void buzzerOn(){ digitalWrite(BUZZER_PIN, HIGH); }
inline void buzzerOff(){ digitalWrite(BUZZER_PIN, LOW); }

bool waterSensed(){
  int v = digitalRead(WATER_SENSOR_PIN);
  return WATER_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}

#if DEBUG_WATER
void dbgPrintWater(const char* tag){
  int raw = digitalRead(WATER_SENSOR_PIN);
  Serial.printf("[WATER][%s] raw=%d, sensed=%s (ACTIVE_HIGH=%d)\n",
                tag, raw, waterSensed() ? "true" : "false", WATER_ACTIVE_HIGH);
}
#else
inline void dbgPrintWater(const char*){}
#endif

void buzzerAlertTask(void*){
  for(int i=0;i<BUZZ_REPEAT;i++){
    buzzerOn();
    vTaskDelay(pdMS_TO_TICKS(BUZZ_ON_MS));
    buzzerOff();
    if(i < BUZZ_REPEAT-1) vTaskDelay(pdMS_TO_TICKS(BUZZ_OFF_MS));
  }
  vTaskDelete(NULL);
}

void startBuzzerAlertPattern(){
  // 코어1 권장(다른 태스크와 동일 코어로 맞춤)
  xTaskCreatePinnedToCore(buzzerAlertTask, "BuzzerAlert",
                          2048, nullptr, 1, nullptr, 1);
}
void showLowWaterAlert(){
  // 여기서는 메시지 전송 안 함: 완료 화면에서 보낼 것
  g_lowWaterAlertActive = true;   // 완료 단계에서 HMI에 표시할지 판단
  startBuzzerAlertPattern();      // 부저 패턴은 그대로
}


// forward declarations
void handleClient(WiFiClient &client);
void doHoming();
void moveSteps(uint32_t steps, bool forward);
inline void stepPulse();
void runPumpForVolume(int mL);
void switchPage(const String &pageName);
void updateJobQueueDisplay();

// ── Nextion 페이지 전환 ─────────────────────────────────────────────────
void switchPage(const String &pageName) {
  g_currentPage = pageName;                // ★ 현재 페이지 기억
  nextion.print("page " + pageName);
  nextion.write(0xFF);
  nextion.write(0xFF);
  nextion.write(0xFF);
}
// complete 페이지의 "다음 작업" 라벨을 갱신
void updateCompleteNextLabel() {
  if (xSemaphoreTake(jobQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (!jobQueue.empty()) {
      Job nextJob = jobQueue.front(); // peek
      int nextTotal = max(0, nextJob.volume + nextJob.margin);          // ★ 덤 포함
      String txt = nextJob.patient_name + " " + String(nextTotal) + "mL" +
                   (nextJob.isUrgent ? " [긴급]" : "");
      sendToNextion("complete.t2.txt=\"" + txt + "\"");
    } else {
      sendToNextion("complete.t2.txt=\"없음\"");
    }
    xSemaphoreGive(jobQueueMutex);
  }
}

// ── 현재 작업을 큐 맨 앞에 되돌리기 ─────────────────────────────────────────
void requeueFront(const Job& job) {
  if (xSemaphoreTake(jobQueueMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    std::queue<Job> temp;
    temp.push(job);                   // 먼저 현재 작업
    while (!jobQueue.empty()) {
      temp.push(jobQueue.front());    // 기존 큐 뒤에 이어붙임
      jobQueue.pop();
    }
    jobQueue = temp;
    xSemaphoreGive(jobQueueMutex);
  }
  updateJobQueueDisplay();
  updateCompleteNextLabel();
}

// ── 작업 대기열 Nextion 표시 갱신 ────────────────────────────────────────────
void updateJobQueueDisplay() {
  if (xSemaphoreTake(jobQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    std::queue<Job> tmp = jobQueue;
    for (int i = 0; i < 7; i++) {
      String txt = "";
      if (!tmp.empty()) {
        Job j = tmp.front(); tmp.pop();
        String label = (i == 0 ? "1st" :
                        i == 1 ? "2nd" :
                        i == 2 ? "3rd" :
                        String(i+1) + "th");
        String urgentMark = j.isUrgent ? " [긴급]" : "";
        txt = label + " " + j.patient_name + " " + String(j.volume) + "mL" + urgentMark;
      }
      String cmd = "process.t" + String(i+2) + ".txt=\"" + txt + "\"";
      sendToNextion(cmd);
    }
    xSemaphoreGive(jobQueueMutex);
  }
}

// pR 처리 전용 함수
void handleNextionPR() {
  // 완료 확인 상태였다면 리셋
  if (dspState == DSP_WAIT_CONFIRM) {
    dspState = DSP_IDLE;
    pageSwitchedToProcess = false;
  }
  // 항상 "조제준비" 누르면 프로세싱 모드로 진입
  isProcessing = true;
  // 큐에 작업이 남았는지 확인
  xSemaphoreTake(jobQueueMutex, pdMS_TO_TICKS(100));
  bool hasJob = !jobQueue.empty();
  xSemaphoreGive(jobQueueMutex);

  if (!isProcessing && hasJob) {
    // 처음 pR → 분주 시작
    isProcessing = true;
    isDispenseReady = true;
    switchPage("process");
  }
  else if (isProcessing && hasJob) {
    // 완료 후 pR → 다음 분주
    isDispenseReady = true;
    switchPage("process");
  }
  else {
    // 큐 비었으면 키패드로
    isProcessing = false;
    switchPage("keypad");
  }
  updateJobQueueDisplay();
}
void saveStepsPositions() {
  EEPROM.write(ADDR_STEPS_FLAG, STEPS_VALID_FLAG);
  EEPROM.put(ADDR_STEPS30,  g_steps30);
  EEPROM.put(ADDR_STEPS60,  g_steps60);
  EEPROM.put(ADDR_STEPS100, g_steps100);
  EEPROM.put(ADDR_STEPSPG,  g_stepsPg); 
  EEPROM.commit();
}

bool loadStepsPositions() {
  if (EEPROM.read(ADDR_STEPS_FLAG) != STEPS_VALID_FLAG) return false;
  EEPROM.get(ADDR_STEPS30,  g_steps30);
  EEPROM.get(ADDR_STEPS60,  g_steps60);
  EEPROM.get(ADDR_STEPS100, g_steps100);
  EEPROM.get(ADDR_STEPSPG,  g_stepsPg);
  return true;
}

// ── HTTP 서버 태스크 ─────────────────────────────────────────────────────────
void httpServerTask(void* pvParameters) {
  for (;;) {
    WiFiClient client = server.available();
    if (client) handleClient(client);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void scanWiFi() {
  //WiFi.disconnect(true, true);
  //delay(100);
  //WiFi.mode(WIFI_STA);  
  Serial.println("📡 WiFi 스캔 시작");
  int n = WiFi.scanNetworks(false, true);
  Serial.println("📡 스캔 완료, 네트워크 수: " + String(n));
  if (n == 0) {
    sendToNextion("page0.t1.txt=\"No networks\"");
  } else {
    for (int i = 0; i < n && i < 6; i++) {
      ssidList[i] = WiFi.SSID(i);
      sendToNextion("page0.t" + String(i+1) + ".txt=\"" + ssidList[i] + "\"");
      sendToNextion("page0.t" + String(i+1) + ".style=3");
    }
  }
}

// ── HMI(Nextion) 처리 태스크 ─────────────────────────────────────────────────
void hmiTask(void* pvParameters) {
  for (;;) {
    bool handled = false;

    // Nextion으로부터 들어오는 바이트 처리
    while (nextion.available()) {
      uint8_t c = nextion.read();

      // 터치 노이즈 등 제어문자(0x00~0x1F) 무시, 단 종료 바이트(0xFF)는 처리
      if (c < 0x20 && c != 0xFF) {
        ffCount = 0;
        continue;
      }

      if (c == 0xFF) {
        // 0xFF 세 번 연속 수신 시 커맨드 끝
        if (++ffCount == 3) {
          ffCount = 0;

          // 완성된 명령어 파싱
          inputBuffer.trim();
          inputBuffer.replace("\r", "");
          inputBuffer.replace("\n", "");
          inputBuffer.replace("\0", "");

          if (inputBuffer.length() > 0) {
            Serial.println("📩 수신된 명령: " + inputBuffer);
            // ── 여기에 pG 처리 추가 ─────────────────────────
            // ── pH: 단순 홈 복귀 ───────────────────────────────
            if (inputBuffer == "pH") {
              Serial.println("▶ pH 수신: HOMING 실행");
              doHoming();
              inputBuffer = "";
              ffCount = 0;
              continue;
            }
            if (inputBuffer == "pT") {
              g_steps30 = (uint32_t)max(0, (int)g_absSteps);
              saveStepsPositions();
              Serial.println("📌 Saved 30mL pos = " + String(g_steps30) + " steps");
              inputBuffer = ""; ffCount = 0; continue;
            }
            if (inputBuffer == "pY") {
              g_steps60 = (uint32_t)max(0, (int)g_absSteps);
              saveStepsPositions();
              Serial.println("📌 Saved 60mL pos = " + String(g_steps60) + " steps");
              inputBuffer = ""; ffCount = 0; continue;
            }
            if (inputBuffer == "pO") {  
              g_steps100 = (uint32_t)max(0, (int)g_absSteps);
              saveStepsPositions();
              Serial.println("📌 Saved 100mL pos = " + String(g_steps100) + " steps");
              inputBuffer = ""; ffCount = 0; continue;
            }
            if (inputBuffer == "pM") {
              g_stepsPg = (uint32_t)max(0, (int)g_absSteps);
              saveStepsPositions();
              Serial.println("📌 Saved pG target = " + String(g_stepsPg) + " steps");
              inputBuffer = ""; ffCount = 0; continue;
            }
            if (inputBuffer == "pC") {
              Serial.println("▶ pC 수신: 100mL 위치 분주(5초) 시퀀스 실행");

              // 안전장치: 현재 자동 분주 중이면 무시 (원하면 큐에 넣도록 변경 가능)
              if (dspState != DSP_IDLE && dspState != DSP_WAIT_CONFIRM) {
                Serial.println("⚠ 현재 작업 중이므로 pC 시퀀스 무시");
                inputBuffer = "";
                ffCount = 0;
                continue;
              }

              // 1) 홈 복귀
              doHoming();

              // 2) 100mL 병 위치로 이동 (기존 로직에서 100mL는 STEPS3 사용)
              moveSteps(g_steps100, true);

              // 3) 5초간 펌프 구동
              runPumpForMs(5000);

              // 4) 홈 복귀
              doHoming();

              // 필요시 HMI 표시 업데이트(선택)
              // sendToNextion("complete.t0.txt=\"100mL  5s dispense\"");

              inputBuffer = "";
              ffCount = 0;
              continue;
            }
            if (inputBuffer == "pX") {
              Serial.println("▶ pX 수신: 60mL 위치 분주(3초) 시퀀스 실행");

              // 안전장치: 현재 자동 분주 중이면 무시 (원하면 큐에 넣도록 변경 가능)
              if (dspState != DSP_IDLE && dspState != DSP_WAIT_CONFIRM) {
                Serial.println("⚠ 현재 작업 중이므로 pX 시퀀스 무시");
                inputBuffer = "";
                ffCount = 0;
                continue;
              }

              // 1) 홈 복귀
              doHoming();

              // 2) 60mL 병 위치로 이동 (기존 로직에서 60mL는 STEPS2 사용)
              moveSteps(g_steps60, true);

              // 3) 3초간 펌프 구동
              runPumpForMs(3000);

              // 4) 홈 복귀
              doHoming();

              // 필요시 HMI 표시 업데이트(선택)
              // sendToNextion("complete.t0.txt=\"100mL  5s dispense\"");

              inputBuffer = "";
              ffCount = 0;
              continue;
            }
            if (inputBuffer == "pE") {
              Serial.println("▶ pE 수신: 30mL 위치 분주(1초) 시퀀스 실행");

              // 안전장치: 현재 자동 분주 중이면 무시 (원하면 큐에 넣도록 변경 가능)
              if (dspState != DSP_IDLE && dspState != DSP_WAIT_CONFIRM) {
                Serial.println("⚠ 현재 작업 중이므로 pE 시퀀스 무시");
                inputBuffer = "";
                ffCount = 0;
                continue;
              }

              // 1) 홈 복귀
              doHoming();

              // 2) 30mL 병 위치로 이동 (기존 로직에서 30mL는 STEPS1 사용)
              moveSteps(g_steps30, true);

              // 3) 1초간 펌프 구동
              runPumpForMs(1000);

              // 4) 홈 복귀
              doHoming();

              // 필요시 HMI 표시 업데이트(선택)
              // sendToNextion("complete.t0.txt=\"100mL  5s dispense\"");

              inputBuffer = "";
              ffCount = 0;
              continue;
            }
            
            if (inputBuffer == "pG" || inputBuffer == "pg") {
              Serial.println("▶ pG 수신: HOMING 후 저장 위치로 이동 (" + String(g_stepsPg) + " steps)");
              doHoming();
              moveSteps(g_stepsPg, true);
              inputBuffer = ""; ffCount = 0; continue;
            }
            if (inputBuffer == "pS") {
              Serial.println("▶ pS 수신: DC 펌프 작동 시작");
                          
              dbgPrintWater("pS_before");     // ★ 추가
              digitalWrite(PUMP_EN, HIGH);
              digitalWrite(PUMP_PWM, HIGH);
              dbgPrintWater("pS_after");      // ★ 추가
              inputBuffer = "";
              ffCount = 0;
              continue;
            }
            else if (inputBuffer == "pI") {
              Serial.println("■ pP 수신: DC 펌프 작동 정지");
              digitalWrite(PUMP_PWM, LOW);
              digitalWrite(PUMP_EN, LOW);
              inputBuffer = "";
              ffCount = 0;
              continue;
            }
            if (inputBuffer == "pR") {
              Serial.println("🔁 pR 수신");
              handleNextionPR();
                // ★ HMI 경고 메시지와 플래그 초기화
              sendToNextion("complete.t1.txt=\"\"");
              g_lowWaterAlertActive = false;
              inputBuffer = "";
              ffCount = 0;
            }
            // ── 조그: pV(정방향 시작), pB(역방향 시작), pN(정지) ──────────────────────
            if (inputBuffer == "pV") {       // forward press
              if (dspState == DSP_IDLE || dspState == DSP_WAIT_CONFIRM) g_jogDir = 1;
              inputBuffer = ""; ffCount = 0; continue;
            }
            if (inputBuffer == "pB") {       // backward press
              if (dspState == DSP_IDLE || dspState == DSP_WAIT_CONFIRM) g_jogDir = 2;
              inputBuffer = ""; ffCount = 0; continue;
            }
            if (inputBuffer == "pN") {       // button release → stop
              g_jogDir = 0;
              inputBuffer = ""; ffCount = 0; continue;
            }

            // --- pU : 목표 분주량 설정 ---
            else if (inputBuffer.endsWith("U")) {
              U_volume = inputBuffer.substring(1, inputBuffer.length() - 1).toInt();
              Serial.println("📐 목표 분주량: " + String(U_volume) + " mL");
            }
            // --- pD : 속도 설정 (mL/5s → mL/s) ---
            else if (inputBuffer.endsWith("D")) {
              int x = inputBuffer.substring(1, inputBuffer.length() - 1).toInt();
              rateFlag = x;
              rate_mL_per_sec = (x > 0) ? (float)x / 5.0f : 0;
              saveFlagsToEEPROM();
              Serial.println("⚖️ 속도: " + String(rate_mL_per_sec, 2) + " mL/s");
            }
            // --- pS : 오프셋 설정 ---
            else if (inputBuffer.endsWith("S")) {
              S_offset = inputBuffer.substring(1, inputBuffer.length() - 1).toInt();
              Serial.println("➕ Offset: " + String(S_offset) + " mL");
            }
            // --- pF : WiFi 리스트 새로고침 ---
            else if (inputBuffer == "pF") {
              Serial.println("♻️ WiFi 리스트 새로고침 요청");
              scanWiFi();
            }
            // --- pP : 수동 분주 시작 ---
            else if (inputBuffer == "pP") {
              Job newJob = { U_volume, marginFlag, "수동조제", false };
              if (xSemaphoreTake(jobQueueMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                jobQueue.push(newJob);
                xSemaphoreGive(jobQueueMutex);
              }
              Serial.println("✅ 수동 작업 대기열 추가");
                // 2) 현재 아무 작업도 안 돌고 있으면(=키패드 화면) 즉시 시작
              if (!isProcessing) {
                // 분주 준비 플래그 세팅
                isProcessing    = true;
                isDispenseReady = true;
                // PROCESS 화면으로 전환
                switchPage("process");
              }
              updateCompleteNextLabel();
            }
            // --- A/F/G/B: 설정 저장 (100/60/30mL 속도, margin) ---
            else if (inputBuffer.endsWith("A")) {
              rate100Flag = inputBuffer.substring(1, inputBuffer.length() - 1).toInt();
              saveFlagsToEEPROM();
              sendToNextion("v100.n1.val=" + String(rate100Flag));
              Serial.println("💾 100 mL 속도 저장: " + String(rate100Flag));
            }
            else if (inputBuffer.endsWith("B")) {
              rate60Flag = inputBuffer.substring(1, inputBuffer.length() - 1).toInt();
              saveFlagsToEEPROM();
              sendToNextion("v60.n1.val=" + String(rate60Flag));
              Serial.println("💾 60 mL 속도 저장: " + String(rate60Flag));
            }
            // ── hmiTask() 내부, pSSID/password 처리 바로 아래에 추가 ────────────────────
            else if (inputBuffer == "pWIFI") {
              // "연결" 버튼이 눌렸을 때 이 명령이 들어온다고 가정
              Serial.println("🔑 pWIFI 수신 → 실제 WiFi 연결 시도");
              connectToWiFi();
              // 초기화
              readyToConnect = false;
            }
            
            else if (inputBuffer.endsWith("G")) {
              rate30Flag = inputBuffer.substring(1, inputBuffer.length() - 1).toInt();
              saveFlagsToEEPROM();
              sendToNextion("v30.n1.val=" + String(rate30Flag));
              Serial.println("💾 30 mL 속도 저장: " + String(rate30Flag));
            }
            else if (inputBuffer.endsWith("W")) {
              marginFlag = inputBuffer.substring(1, inputBuffer.length() - 1).toInt();
              saveFlagsToEEPROM();
              sendToNextion("vol.n1.val=" + String(marginFlag));
              Serial.println("💾 Margin 저장: " + String(marginFlag));
            }
            // --- SSID 선택 / 비번 입력 처리 ---
            else if (inputBuffer.startsWith("pSSID")) {
              int idx = inputBuffer.substring(5).toInt();
              if (idx >= 1 && idx <= 6) {
                selectedSSID = ssidList[idx - 1];
                Serial.println("🔑 SSID 선택: " + selectedSSID);
              }
            }
            else if (inputBuffer.endsWith("password")) {
              wifiPassword = inputBuffer.substring(1, inputBuffer.length() - 8);
              wifiPassword.trim();
              readyToConnect = true;
              Serial.println("🔐 Password 입력 완료");
            }
          } // if inputBuffer.length > 0

          // 버퍼 초기화
          inputBuffer = "";
        } // if ffCount == 3
      }
      else {
        // 0xFF가 아닌 데이터 바이트는 명령어 버퍼에 저장
        inputBuffer += (char)c;
        ffCount = 0;
      }

      if (handled) break;
    } // while nextion.available()

    // 짧게 대기하여 다른 태스크에 CPU 양보
    vTaskDelay(pdMS_TO_TICKS(10));
  } // for(;;)
}

void dispenseTask(void* parameter) {
  while (true) {
    // 1) 준비 신호(pS) 올 때까지 대기
    if (!isDispenseReady) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // 2) 큐에서 작업 꺼내기
    Job currentJob;
    bool hasJob = false;
    if (xSemaphoreTake(jobQueueMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!jobQueue.empty()) {
        currentJob = jobQueue.front();
        jobQueue.pop();
        hasJob = true;
      }
      xSemaphoreGive(jobQueueMutex);
    }

    if (!hasJob) {
      // 작업이 없으면 다음 준비 신호를 기다림
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    // 준비 신호 소비
    isDispenseReady = false;
    // 3) 실제 분주 로직 시작
    U_volume = currentJob.volume;
    S_offset = currentJob.margin;
    switchPage("process");
        // ← 이 줄 다음에 추가
        // ★ 덤 포함 총량으로 표시
    int total_mL = max(0, U_volume + S_offset);
    sendToNextion(
      "process.g0.txt=\"" +
      currentJob.patient_name +
      "  " +
      String(total_mL) +
      "mL" +
      (currentJob.isUrgent ? " [긴급]" : "") +
      "\""
    );
    dspState = DSP_HOMING;
    updateJobQueueDisplay();
    while (dspState != DSP_IDLE && dspState != DSP_WAIT_CONFIRM) {
      switch (dspState) {
        case DSP_HOMING:
          doHoming();
          dspTimer = millis();
          dspState = DSP_HOMED_WAIT;
          break;

        case DSP_HOMED_WAIT:
          if (millis() - dspTimer >= 200) dspState = DSP_MOVE;
          break;

        case DSP_MOVE: {
            int total_mL = max(0, U_volume + S_offset);         // ★ 기본 + 덤
            uint32_t steps = (total_mL <= 30 ? g_steps30
                              : total_mL <= 60 ? g_steps60
                                               : g_steps100);
            moveSteps(steps, true);
            dspTimer = millis();
            dspState = DSP_MOVE_WAIT;
          }
          break;

        case DSP_MOVE_WAIT:
          if (millis() - dspTimer >= 500) {
            dbgPrintWater("FSM_before");

            // 병 감지 신호: LOW이면 병이 있음(현재 배선/로직 기준)
            bool bottleDetected = (digitalRead(SENSOR_D0_PIN) == LOW);

            if (bottleDetected) {
              int total_mL = max(0, U_volume + S_offset); // ★ 기본 + 덤
              float speed = (total_mL <= 30 && rate30Flag > 0) ? rate30Flag / 1.0f
                            : (total_mL <= 60 && rate60Flag > 0) ? rate60Flag / 3.0f
                                                                  : rate100Flag / 5.0f;
              dspPumpDuration = (uint32_t)(total_mL / speed * 1000.0f);

              // 미감지면 경고(물 센서) 로직은 유지
              if (!waterSensed()) showLowWaterAlert();

              digitalWrite(PUMP_EN, HIGH);
              digitalWrite(PUMP_PWM, HIGH);
              dbgPrintWater("FSM_after");
              dspTimer = millis();
              dspState = DSP_PUMP_WAIT;
            } else {
              // ★★★ 병 미감지: 현재 작업을 '완료로 처리하지 않고' 재시도 대기 상태로 전환 ★★★
              // 1) 부저 3회
              startBuzzerAlertPattern();
              doHoming();
              // 2) 화면 전환 및 안내 메시지
              switchPage("complete");
              sendToNextion("complete.t1.txt=\"시럽병을 설치하세요\"");

              // 3) 현재 작업을 큐 맨 앞에 되돌림 (retry)
              requeueFront(currentJob);

              // 4) 사용자의 재시작(pR)을 기다리도록 완료대기 상태로 전환
              dspState = DSP_WAIT_CONFIRM;
            }
          }
          break;

        case DSP_PUMP_WAIT:
          if (millis() - dspTimer >= dspPumpDuration) {
            digitalWrite(PUMP_PWM, LOW);
            digitalWrite(PUMP_EN, LOW);
            //시럽이 다 떨어질 수 있도록 1초 대기 (FreeRTOS 방식)
            vTaskDelay(pdMS_TO_TICKS(1000));  // 1000ms = 1초
            dspState = DSP_RETURN;
          }
          break;

        case DSP_RETURN:
          doHoming();
          dspTimer = millis();
          dspState = DSP_RETURN_WAIT;
          break;

        case DSP_RETURN_WAIT:
          if (millis() - dspTimer >= 200) dspState = DSP_COMPLETE;
          break;

        case DSP_COMPLETE:
          switchPage("complete");
          // complete.t0.txt에 "환자명  총량mL" 표시
          int total_mL = max(0, U_volume + S_offset);   // ★ 실제 분주량

          sendToNextion(
            "complete.t0.txt=\"" +
            currentJob.patient_name +
            "  " +
            String(total_mL) +
            "mL" +
            (currentJob.isUrgent ? " [긴급]" : "") +
            "\""
          );          
          updateCompleteNextLabel();
            // ★ 수위 경고가 이번 사이클에 있었으면 메시지 표시
          if (g_lowWaterAlertActive) {
            sendToNextion("complete.t1.txt=\"시럽 잔량이 부족합니다\"");
          }
          dspState = DSP_WAIT_CONFIRM;
          break;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      
    }
  }
}

// ===== Part 2: 헬퍼 함수, handleClient(), setup(), loop() =====

// ── homing ───────────────────────────────────────────────────
void doHoming() {
  pinMode(LIMIT_PIN, INPUT);   // 외부 6.8kΩ로 3.3V에 풀업했기 때문에 INPUT
  while (!homeTriggered()) {   // 트리거될 때까지 이동
    digitalWrite(EN_PIN, LOW);
    digitalWrite(DIR_PIN, HIGH);
    stepPulse();
  }
  digitalWrite(EN_PIN, HIGH);
  delay(200);
  g_absSteps = 0;   // ★ 홈 기준 0
}

// ── 스텝 구동 ───────────────────────────────────────────────
void moveSteps(uint32_t steps, bool forward) {
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, forward?LOW:HIGH);
  for (uint32_t i=0;i<steps;i++) {
    stepPulse();
    if (forward) g_absSteps++;
    else if (g_absSteps > 0) g_absSteps--;  // 0 미만 방지
  }

  digitalWrite(EN_PIN, HIGH);
}

// ── 펄스 ────────────────────────────────────────────────────
inline void stepPulse() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_US);
}

// ── 펌프 제어 ───────────────────────────────────────────────
void runPumpForVolume(int mL) {
  float speed = (mL<=30&&rate30Flag>0?rate30Flag/5.0f:
                 mL<=60&&rate60Flag>0?rate60Flag/5.0f:
                 rate100Flag>0?rate100Flag/5.0f:1.0f);
  uint32_t ms = (uint32_t)((float)mL/speed*1000.0f);
  dbgPrintWater("runPumpForVolume_before");  // ★ 추가
  digitalWrite(PUMP_EN, HIGH);
  digitalWrite(PUMP_PWM, HIGH);
  dbgPrintWater("runPumpForVolume_after");   // ★ 추가
  delay(ms);
  digitalWrite(PUMP_EN, LOW);
  digitalWrite(PUMP_PWM, LOW);
}

// ── 펌프를 지정 시간(ms) 동안 구동 ─────────────────────────────────────────
void runPumpForMs(uint32_t ms) {
  digitalWrite(PUMP_EN, HIGH);
  digitalWrite(PUMP_PWM, HIGH);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    vTaskDelay(pdMS_TO_TICKS(10)); // 다른 태스크에 양보
  }
  digitalWrite(PUMP_PWM, LOW);  // OFF
  digitalWrite(PUMP_EN, LOW);   // OFF
  // 2) 시럽 드립 떨어질 시간 1초 확보
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ── EEPROM 유틸 ────────────────────────────────────────────
void saveNetworkCredentials(const String &ssid,const String &pass){
  for(int i=0;i<32;i++){
    EEPROM.write(ADDR_SSID+i,     i<ssid.length()?ssid[i]:0);
    EEPROM.write(ADDR_PASSWORD+i, i<pass.length()?pass[i]:0);
  }
  EEPROM.write(ADDR_FLAG, VALID_FLAG);
  EEPROM.commit();
}
bool loadNetworkCredentials(String &ssid,String &pass){
  if(EEPROM.read(ADDR_FLAG)!=VALID_FLAG) return false;
  char s[33],p[33];
  for(int i=0;i<32;i++){ s[i]=EEPROM.read(ADDR_SSID+i); p[i]=EEPROM.read(ADDR_PASSWORD+i); }
  s[32]=p[32]=0; ssid=String(s); ssid.trim(); pass=String(p); pass.trim();
  return true;
}
void saveFlagsToEEPROM(){
  EEPROM.put(ADDR_VOLUME, volumeFlag);
  EEPROM.put(ADDR_MARGIN, marginFlag);
  EEPROM.put(ADDR_RATE100,rate100Flag);
  EEPROM.put(ADDR_RATE60, rate60Flag);
  EEPROM.put(ADDR_RATE30, rate30Flag);
  EEPROM.commit();
}
void loadFlagsFromEEPROM(){
  EEPROM.get(ADDR_VOLUME, volumeFlag);
  EEPROM.get(ADDR_MARGIN, marginFlag);
  EEPROM.get(ADDR_RATE100,rate100Flag);
  EEPROM.get(ADDR_RATE60, rate60Flag);
  EEPROM.get(ADDR_RATE30, rate30Flag);
}

// ── 헬퍼 ───────────────────────────────────────────────────
String getMacAddressString(){
  uint8_t mac[6]; WiFi.macAddress(mac);
  char buf[18];
  sprintf(buf,"%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return String(buf);
}
void sendToNextion(const String &cmd){
  nextion.print(cmd);
  nextion.write(0xFF); nextion.write(0xFF); nextion.write(0xFF);
}
void jogTask(void* pvParameters) {
  for (;;) {
    // 분주 중이면 조그 금지
    if (!(dspState == DSP_IDLE || dspState == DSP_WAIT_CONFIRM)) {
      g_jogDir = 0;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (g_jogDir == 0) {      // 버튼 안 눌림
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    // 조그 시작
    bool forward = (g_jogDir == 1);
    digitalWrite(EN_PIN, LOW);
    digitalWrite(DIR_PIN, forward ? LOW : HIGH);

    uint16_t us = JOG_US_START;   // 부드럽게 출발(가속 램프)
    uint32_t cnt = 0;

    // 버튼을 누르는 동안 연속 펄스 출력
    while (g_jogDir != 0) {
      // 반대방향으로 바뀌면 재설정 위해 빠져나감
      if ((forward && g_jogDir != 1) || (!forward && g_jogDir != 2)) break;

      // 홈쪽으로 갈 때는 센서 안전
      if (!forward && homeTriggered()) break;

      stepPulseJogVariable(us);
      if (forward) g_absSteps++;
      else if (g_absSteps > 0) g_absSteps--;


      // 가속: us를 TARGET까지 서서히 줄임(half-period ↓ → 속도 ↑)
      if (us > JOG_US_TARGET) {
        uint16_t next = us > JOG_ACCEL_PER_STEP ? us - JOG_ACCEL_PER_STEP : JOG_US_TARGET;
        us = (next < JOG_US_TARGET) ? JOG_US_TARGET : next;
      }

      // 다른 태스크에 주기적으로 양보
      if ((++cnt & 0xFF) == 0) vTaskDelay(1);
    }

    // (선택) 부드러운 정지: 버튼에서 손 떼면 잠깐 감속 펄스 몇 개 더
    // for (uint16_t s = 0; s < 100; ++s) {
    //   us += 3; if (us > 900) us = 900;
    //   stepPulseJogVariable(us);
    // }

    digitalWrite(EN_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ── setup() 위쪽에 추가 ────────────────────────────────────────────────
void connectToWiFi() {
  Serial.println();
  Serial.println("🚀 WiFi 연결 시도: " + selectedSSID);

  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(selectedSSID.c_str(), wifiPassword.c_str());

  // 최대 10초 대기
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("✅ 연결 성공, IP: " + WiFi.localIP().toString());
    // ← 여기에 EEPROM 저장
    saveNetworkCredentials(selectedSSID, wifiPassword);
    Serial.println("💾 SSID·PW를 EEPROM에 저장했습니다");    
    // Nextion에 연결 성공 표시
    sendToNextion("page0.g0.txt=\"Connected to " + selectedSSID + "\"");
    sendToNextion("page0.t8.txt=\"" + WiFi.localIP().toString() + "\"");
    // HTTP 서버 시작
    server.begin();
    Serial.println("▶️ HTTP 서버 시작됨");
  } else {
    Serial.println();
    Serial.println("❌ 연결 실패");
    sendToNextion("page0.g0.txt=\"Connection failed\"");
  }
}

// ── HTTP 요청 처리 ─────────────────────────────────────────
void handleClient(WiFiClient &client){
  String req = client.readStringUntil('\r');
  client.read(); req.trim();
  Serial.println("🌐 요청: "+req);

  if(req.startsWith("GET / ")){
    String body = "{\"status\":\"ready\",\"mac\":\""+getMacAddressString()+
                  "\",\"ip\":\""+WiFi.localIP().toString()+"\"}";
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.print("Content-Length: "); client.println(body.length());
    client.println("Connection: close");
    client.println(); client.print(body);
    client.stop();
    return;
  }

    // ── POST /dispense 처리 ───────────────────────────────────────────────
  if (req.startsWith("POST /dispense")) {
      int contentLength = 0;
      while (client.connected()) {
          String h = client.readStringUntil('\r'); client.read();
          h.trim();
          if (h.length() == 0) break;
          if (h.startsWith("Content-Length:"))
              contentLength = h.substring(15).toInt();
      }
      String body;
      while ((int)body.length() < contentLength) {
          if (client.available()) body += char(client.read());
      }
      Serial.println("📥 JSON: " + body);

      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, body);
      if (!err && doc.containsKey("total_volume")) {
          int vol = doc["total_volume"];
          String name = doc["patient_name"] | "Unknown";
          bool urgent = doc["urgent"] | false;  // 긴급 플래그 추가
          Serial.println("📥 환자: " + name + ", vol=" + String(vol) + ", urgent=" + String(urgent));

          // 1) 큐에 추가 (긴급 작업은 맨 앞에 추가)
          xSemaphoreTake(jobQueueMutex, pdMS_TO_TICKS(100));
          
          if (urgent) {
            // 긴급 작업: 기존 큐를 임시로 복사하고, 긴급 작업을 맨 앞에 넣은 후 다시 복사
            std::queue<Job> tempQueue;
            tempQueue.push({vol, marginFlag, name, true});  // 긴급 작업을 맨 앞에
            
            // 기존 작업들을 그 뒤에 추가
            while (!jobQueue.empty()) {
              tempQueue.push(jobQueue.front());
              jobQueue.pop();
            }
            
            // 임시 큐를 원래 큐로 복사
            jobQueue = tempQueue;
            Serial.println("🚨 긴급 작업이 대기열 맨 앞에 추가됨");
          } else {
            // 일반 작업: 맨 뒤에 추가
            jobQueue.push({vol, marginFlag, name, false});
          }
          
          size_t qsize = jobQueue.size();
          xSemaphoreGive(jobQueueMutex);
          updateCompleteNextLabel();

          // 2) PROCESS 화면 및 대기열 업데이트
          //switchPage("process");
          sendToNextion("process.n0.val=" + String(qsize));
          updateJobQueueDisplay();
          // ★ 추가: 현재 분주 중이 아니면(=키패드에 있을 때) 자동으로 첫 작업 시작
          if (!isProcessing) {
            isProcessing    = true;
            isDispenseReady = true;
            //switchPage("process");
          }
          if (isProcessing && dspState == DSP_IDLE) {
            isDispenseReady = true;
            //switchPage("process");
          }          
          // 3) 유휴 상태면 즉시 시작
          //if (dspState == DSP_IDLE) {
              //isDispenseReady = true;
          //}
          updateJobQueueDisplay();
          // 4) 응답
          String res = (dspState == DSP_IDLE ? "OK" : "BUSY");
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/plain");
          client.print("Content-Length: "); client.println(res.length());
          client.println("Connection: close");
          client.println(); client.print(res);
          client.stop();

          Serial.println("✅ 대기열 추가됨, 응답: " + res);
          return;
      }

      // 잘못된 JSON
      client.println("HTTP/1.1 400 Bad Request");
      client.println("Content-Type: text/plain");
      client.print("Content-Length: 12"); client.println();
      client.print("Invalid JSON");
      client.stop();
      return;
  }

    String nf="404 Not Found";
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain");
    client.print("Content-Length: "); client.println(nf.length());
    client.println("Connection: close");
    client.println(); client.print(nf);
    client.stop();
  }

// ── setup() ────────────────────────────────────────────────────────────────
void setup(){
  Serial.begin(115200); delay(1000);
  Serial.println("🚀 ESP32 부팅");
  // TMC2209 init
  pinMode(EN_PIN,OUTPUT); digitalWrite(EN_PIN,HIGH);
  pinMode(DIR_PIN,OUTPUT); pinMode(STEP_PIN,OUTPUT);
  pinMode(PUMP_EN,OUTPUT); pinMode(PUMP_PWM,OUTPUT);
  digitalWrite(PUMP_EN,LOW); digitalWrite(PUMP_PWM,LOW);
  pinMode(SENSOR_D0_PIN,INPUT_PULLUP);
  pinMode(LIMIT_PIN, INPUT);   // 광센서 외부 풀업환경 → INPUT
  TMCserial.begin(115200, SERIAL_8N1, -1, UART_TX);  // rxPin=-1(미사용), txPin=17
  driver.begin();
  driver.rms_current(1200,0.0);
  driver.pdn_disable(true);        // PDN 핀 기능 비활성 → UART 우선
  driver.mstep_reg_select(true);   // MS1/MS2 핀 무시, 레지스터 MRES 사용
  driver.microsteps(16);           // 1/16 (원하는 값으로: 2/4/8/16/32/64/128/256)
  driver.intpol(true);             // 256분해능 인터폴레이션(가능한 버전에서)
  driver.pwm_autoscale(true);      // (권장) 스텔스촙 튜닝에 도움
  driver.en_spreadCycle(false);
  driver.TPOWERDOWN(10);
  nextion.begin(9600,SERIAL_8N1,NEXTION_RX,NEXTION_TX);
  EEPROM.begin(512); delay(500);
  loadFlagsFromEEPROM();
  sendToNextion("complete.t1.txt=\"\"");   // ★ 경고 텍스트 초기화
  g_lowWaterAlertActive = false;          // ★ 플래그도 초기화

    // Water sensor / Buzzer
  #if WATER_USE_INTERNAL_PULLUP
    pinMode(WATER_SENSOR_PIN, INPUT_PULLUP);
  #else
    pinMode(WATER_SENSOR_PIN, INPUT);
  #endif
    pinMode(BUZZER_PIN, OUTPUT);
    buzzerOff(); // 부팅 시 무음
  #if DEBUG_WATER
    Serial.printf("WATER pin=%d, mode=%s, ACTIVE_HIGH=%d\n",
                  WATER_SENSOR_PIN,
                  WATER_USE_INTERNAL_PULLUP ? "INPUT_PULLUP" : "INPUT",
                  WATER_ACTIVE_HIGH);
    dbgPrintWater("setup");
  #endif

  if (loadStepsPositions()) {
  Serial.println("📥 Pos from EEPROM: 30=" + String(g_steps30) +
                 ", 60=" + String(g_steps60) +
                 ", 100=" + String(g_steps100));
  } else {
  Serial.println("ℹ️ Pos default used (no EEPROM calib)");
  }

  U_volume=volumeFlag; S_offset=marginFlag;
  rate_mL_per_sec=(rateFlag>0)?(float)rateFlag/5.0f:1.0f;
  sendToNextion("v100.n1.val="+String(rate100Flag));
  sendToNextion("vol.n1.val="+String(marginFlag));
  sendToNextion("v60.n1.val="+String(rate60Flag));
  sendToNextion("v30.n1.val="+String(rate30Flag));

  String ss,sq;
  if(loadNetworkCredentials(ss,sq)){
    selectedSSID=ss; wifiPassword=sq;
    Serial.println("📶 SSID:"+selectedSSID);
    Serial.println("🔐 PW:"+wifiPassword);
    WiFi.disconnect(true,true);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.begin(selectedSSID.c_str(),wifiPassword.c_str());
    unsigned long t0=millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-t0<10000){
      delay(500); Serial.print(".");
    }
    Serial.println();
    if(WiFi.status()==WL_CONNECTED){
      Serial.println("✅ 연결:"+selectedSSID);
      Serial.println("🌐 IP:"+WiFi.localIP().toString());
      sendToNextion("page0.g0.txt=\"Connected to "+selectedSSID+"\"");
      sendToNextion("page0.t8.txt=\""+WiFi.localIP().toString()+"\"");
      server.begin();
    } else {
      Serial.println("❌ WiFi 실패");
      sendToNextion("page0.g0.txt=\"Connection failed\"");
      WiFi.disconnect(true,true);
      WiFi.mode(WIFI_OFF);
      delay(100);
      WiFi.mode(WIFI_STA);
    }
  } else {
    Serial.println("📭 WiFi 정보 없음");
  }
  scanWiFi();
  // FreeRTOS 태스크 생성
  jobQueueMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    dispenseTask,
    "DispenseTask",
    8192,
    NULL,
    1,
    &dispenseTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    httpServerTask,
    "HTTPServerTask",
    8192,
    NULL,
    2,
    &httpTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    hmiTask,
    "HMITask",
    4096,
    NULL,
    1,
    &hmiTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    jogTask,
    "JogTask",
    4096,
    NULL,
    1,
    &jogTaskHandle,
    1
  );

  switchPage("confirm");
}

void loop(){
  static unsigned long lastHMIUpdate = 0;
  static int lastQueueSizeShownOnComplete = -1;   // ★ 추가

  if (millis() - lastHMIUpdate >= 500) {
    lastHMIUpdate = millis();

    // 기존 process 페이지 카운터 갱신
    if (xSemaphoreTake(jobQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      int qsize = (int)jobQueue.size();
      xSemaphoreGive(jobQueueMutex);
      sendToNextion("process.n0.val=" + String(qsize));
      updateJobQueueDisplay();

      // ★ complete 페이지가 열려 있고, 대기열 크기 변화가 있으면 라벨 갱신
      if (g_currentPage == "complete") {
        if (qsize != lastQueueSizeShownOnComplete) {
          updateCompleteNextLabel();
          lastQueueSizeShownOnComplete = qsize;
        }
      } else {
        // 다른 페이지면 트래킹만 초기화
        lastQueueSizeShownOnComplete = -1;
      }
    }
  }
  delay(10);
}
