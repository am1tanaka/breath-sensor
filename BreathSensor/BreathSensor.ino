/**
 * ブレスコールセンサープログラム
 * 
 * SENSOR1/SENSOR2 = フォトトランジスタの値(0〜1023)
 * 
 * githubに登録
 * 
 * @copyright 2016 YuTanaka@AmuseOne
 */

#include <Servo.h>
#include <EEPROM.h>

/** SERIAL_ENABLED = trueの時、読み取った値をシリアルで送信する*/
const bool SERIAL_ENABLED = true;

/** EEPROMへの作動回数カウントアップを有効にする(EEPROMは寿命があるので、不要になったらfalseにする*/
const bool EEPROM_ENABLED = true && SERIAL_ENABLED;
/** 電圧低下時の表示*/
const bool VOLT_LOW_ENABLED = false && SERIAL_ENABLED;

/** センサー値を出力*/
const bool SENSOR_PRINT_ENABLED = false && SERIAL_ENABLED;

/** 電圧の表示*/
const bool VOLT_ENABLED = false && SERIAL_ENABLED;

/** 回転とみなす変化の閾値*/
const int THRESHOLD = 40;

/** 停止時の減衰パーセント*/
const float MOTOR_STOP_RATE = 70;
/** モーターを発動する値*/
const float MOTOR_START = 18;
/** 加算値*/
float nowSum = 0;

/** ループ時の待ち時間*/
const unsigned long DELAY_LOOP = 1;

// フォトトランジスタのアナログPIN
const int SENSOR1 = 0;
const int SENSOR2 = 2;
const int SENSOR_VOLT = 4;

// サーボのピン番号(デジタル)
const int SERVO_PIN = 9;
// ライト(デジタル)
const int LED = 13;


// サーボのインスタンス
Servo servo;

/** モーターの発動をキャンセルするms*/
const unsigned long MOTOR_IGNORE_MS = 5000;

/** モーターの角度*/
const int MOTOR_MAX = 160;

/** 現在のモーターの位置*/
int nowMotor = 0;


// 電圧チェック
/** この電圧を下回ったら、センサーを一定時間停止させる*/
const int VOLTAGE_THRESHOLD = 1000;
/** 検出停止ms*/
const unsigned long SENSOR_IGNORE_MS = 200;
/** 最大の停止時間*/
const unsigned long SENSOR_IGNORE_MAX = 2000;
/** この値より電圧が低い時は、停止はしないが加算もしない*/
const unsigned long SENSOR_IGNORE_TICK = 1012;
/** 1ループごとに減衰させる待ち時間のパーセンテージ*/
const unsigned long SENSOR_GENSUI = 99;
/** 現在の停止時間*/
unsigned long sensorIgnoreMs = SENSOR_IGNORE_MS;

// 9600 bps
// 1200 byte/s
// 1.2 byte/ms
// 20byte 送りたい
// 20 msかかる
const unsigned long CHECK_MS = 50;

// 動作回数を書き込むアドレス
const int EEADR_MOVECOUNT = 256;

// 動作回数
byte moveCount = 0;

// Serial Command
/** 動作回数を0にするときc*/
const byte CMD_CLEAR = 'c';
/** 動作回数を返すときr*/
const byte CMD_READ = 'r';

/** 2回連続で閾値を超えていないとカウントしないためのフラグ*/
bool isLastSensor = false;

/** セットアップの完了待ち*/
volatile bool doneSetup = false;

/** 移動平均回数*/
const int IDO_AVG_MAX = 3;

/** 移動平均配列*/
int s1[IDO_AVG_MAX];

/** 現在の移動平均インデックス*/
int idoIndex = 0;

void setup() {
  // LED13を消す
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // put your setup code here, to run once:
  if (SERIAL_ENABLED) {
    Serial.begin(9600);
  }

  if (EEPROM_ENABLED) {
    moveCount = EEPROM.read(EEADR_MOVECOUNT);
  }

  // サーボの初回動作
  servo.attach(SERVO_PIN);
  servo.write(MOTOR_MAX);
  delay(1000);
  servo.write(nowMotor);
  delay(1000);

  // 移動平均をリセット
  for (int i=0 ; i<IDO_AVG_MAX ; i++) {
    s1[i] = analogRead(SENSOR1);
  }
  doneSetup = true;
}

void loop() {
  if (!doneSetup) {
    return;
  }
  
  int min1 = 1024;
  int max1 = 0;
  int minv = 1024;

  // コマンドをチェック
  byte cmd;
  do {
    cmd = Serial.read();
    if (cmd == CMD_CLEAR) {
      moveCount = 0;
      if (EEPROM_ENABLED) {
        EEPROM.write(EEADR_MOVECOUNT, moveCount);
      }
    }
    else if (cmd == CMD_READ) {
      Serial.println("moveCount="+String(moveCount));
    }
  } while(cmd != 255);

  // 電圧が低い時は、無効時間を設定
  unsigned long st = millis();

  // 20ms=20,000 micro sec=100 loop
  bool lastEntry = false; // 前回データを登録したかフラグ
  while ((millis()-st) < CHECK_MS) {
    // voltage
    int nowv = analogRead(SENSOR_VOLT);
    minv = min(minv, nowv);
    
    // 200 micro sec
    s1[idoIndex] = analogRead(SENSOR1);
    int s1ido = idoAvg();
    min1 = min(s1ido, min1);
    max1 = max(s1ido, max1);
    idoIndex = (idoIndex < IDO_AVG_MAX-1) ? idoIndex+1 : 0;
  }

  // センサーの停止をチェック
  sensorIgnoreMs = sensorIgnoreMs*SENSOR_GENSUI/100;
  sensorIgnoreMs = max(sensorIgnoreMs, SENSOR_IGNORE_MS);
  //// 電源低下を確認
  if (minv < VOLTAGE_THRESHOLD) {
    if (VOLT_LOW_ENABLED) {
      dispData(min1, max1, minv);
      Serial.println("delay:"+String(sensorIgnoreMs));
    }

    digitalWrite(LED, LOW);
    isLastSensor = false;
    nowSum = 0;
    delay(sensorIgnoreMs);

    // 待ち時間を増加させる
    sensorIgnoreMs = sensorIgnoreMs*2;
    sensorIgnoreMs = min(sensorIgnoreMs, SENSOR_IGNORE_MAX);
    return;
  }

  // 閾値のオーバーチェック
  bool isNowSensor = false;
  if (minv < SENSOR_IGNORE_TICK) {
    if (VOLT_LOW_ENABLED) {
      Serial.print("tick low: ");
      dispData(min1, max1, minv);
    }
  }
  else if (min1 > 0) 
  {
    int sa = max1-min1;
    isNowSensor = (sa >= THRESHOLD);
  }

  // 今回ONで前回もONの時はLED ON
  if (isLastSensor && isNowSensor) {
    nowSum++;
    digitalWrite(LED, HIGH);
  }
  else {
    digitalWrite(LED, LOW);
    nowSum = nowSum * MOTOR_STOP_RATE / 100.0;
  }
  // 今回の結果を、次回に伝える
  isLastSensor = isNowSensor;

  // モーター発動チェック
  if (nowSum > MOTOR_START) {
    nowSum = 0;           // 値をリセットする
    nowMotor = (nowMotor==0) ? MOTOR_MAX : 0;
    servo.write(nowMotor);
    moveCount++;          // 動作カウントアップ
    if (EEPROM_ENABLED) {
      EEPROM.write(EEADR_MOVECOUNT, moveCount);      
    }
    // モーター動作ご、一定時間は処理を停止
    isLastSensor = false;
    delay(MOTOR_IGNORE_MS);
  }
 
  // 0000-0000/0000-0000 (19byte)
  if (SENSOR_PRINT_ENABLED) {
    dispData(min1, max1, minv);
  }

  // 電圧表示
  if (VOLT_ENABLED) {
    Serial.println("min volt="+String(minv));
  }

  // 処理を一旦離す
  delay(DELAY_LOOP);
}

/** データをシリアルに表示*/
void dispData(int min1,int max1, int minv) {
    String smin1 = "    "+String(min1);
    String smax1 = "    "+String(max1);
    String sa1 = "    "+String(max1-min1);
    String sres = ""
      +String(nowSum)+"/"
      +smin1.substring(smin1.length()-4)+","
      +smax1.substring(smax1.length()-4)+"/"
      +sa1.substring(sa1.length()-4)+"/"
      +String((float)minv*5.0/1023.0);
    Serial.println(sres);
}

/**
 * SENSOR1の移動平均を求める
 */
int idoAvg() {
  int sum = 0;
  int idx = idoIndex;
  for (int i=0 ; i<IDO_AVG_MAX ; i++) {
    sum += s1[idx];
    idx--;
    idx = (idx < 0) ? IDO_AVG_MAX-1 : idx;
  }
  return sum/IDO_AVG_MAX;
}


