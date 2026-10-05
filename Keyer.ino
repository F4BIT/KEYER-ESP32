/*
   ================================================================
                       ESP32-S3 CW KEYER
   ================================================================

   Carte :
     Ulegqin ESP32-S3 DevKitC-1 N16R8

   Arduino ESP32 Core :
     2.0.17

   GPIO VALIDES :
     DIT             GPIO 4
     DAH             GPIO 5
     KEY             GPIO 6
     SIDETONE        GPIO 7
     POT SPEED       GPIO 8
     RGB LED         GPIO 48

   Fonctions :
     - Iambic A
     - Iambic B
     - Paddle memory
     - 5 à 40 WPM
     - Sidetone 700 Hz
     - Key output
     - RGB status LED
     - FIFO CW
     - WinKey basic emulation
     - ASCII -> Morse
     - moteur CW non bloquant

   IMPORTANT :
     Cette version utilise l'ancienne API LEDC compatible
     Arduino ESP32 Core 2.0.17.

   ================================================================
*/

#include <Arduino.h>
#include "driver/rmt.h"
#include "USB.h"
#include "esp_mac.h"

// ================================================================
// GPIO
// ================================================================

#define DIT_PIN        4
#define DAH_PIN        5
#define KEY_PIN        6
#define SIDETONE_PIN   7
#define SPEED_POT_PIN  8

#define RGB_LED_PIN    48

// ================================================================
// USB IDENTITE
// ================================================================

#define USB_VID       0x303A
#define USB_PID       0x4001

#define USB_MANUF     "F4BIT"
#define USB_PRODUCT   "ESP32-S3_CW_Keyer"

// ================================================================
// CW
// ================================================================

#define SIDETONE_FREQ 700

#define MIN_WPM 5
#define MAX_WPM 40

#define IAMBIC_A 0
#define IAMBIC_B 1

// ================================================================
// USB / WinKey
// ================================================================

#define WK_BAUD 1200

// ================================================================
// LED RMT
// ================================================================

#define LED_RMT_CHANNEL RMT_CHANNEL_0

// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns

#define WS_T0H 16
#define WS_T0L 32

#define WS_T1H 32
#define WS_T1L 16

#define WS_RESET 3000

// ================================================================
// MORSE
// ================================================================

struct MorseEntry
{
  char c;
  const char *code;
};

const MorseEntry morseTable[] =
{
  {'A', ".-"},
  {'B', "-..."},
  {'C', "-.-."},
  {'D', "-.."},
  {'E', "."},
  {'F', "..-."},
  {'G', "--."},
  {'H', "...."},
  {'I', ".."},
  {'J', ".---"},
  {'K', "-.-"},
  {'L', ".-.."},
  {'M', "--"},
  {'N', "-."},
  {'O', "---"},
  {'P', ".--."},
  {'Q', "--.-"},
  {'R', ".-."},
  {'S', "..."},
  {'T', "-"},
  {'U', "..-"},
  {'V', "...-"},
  {'W', ".--"},
  {'X', "-..-"},
  {'Y', "-.--"},
  {'Z', "--.."},

  {'0', "-----"},
  {'1', ".----"},
  {'2', "..---"},
  {'3', "...--"},
  {'4', "....-"},
  {'5', "....."},
  {'6', "-...."},
  {'7', "--..."},
  {'8', "---.."},
  {'9', "----."},

  {'.', ".-.-.-"},
  {',', "--..--"},
  {'?', "..--.."},
  {'/', "-..-."},
  {'=', "-...-"},
  {'+', ".-.-."},
  {'-', "-....-"},
  {'(', "-.--."},
  {')', "-.--.-"}
};

const int MORSE_COUNT =
  sizeof(morseTable) / sizeof(morseTable[0]);

// ================================================================
// ETAT GENERAL
// ================================================================

int wpm = 20;

bool iambicMode = true;
uint8_t iambicType = IAMBIC_B;

bool paddleReverse = false;

bool keyDown = false;
bool sidetoneOn = true;

bool ditMemory = false;
bool dahMemory = false;

bool hostOpen = false;
bool paused = false;

// ================================================================
// TIMING CW
// ================================================================

unsigned long ditLength = 60;
unsigned long dahLength = 180;

unsigned long elementStart = 0;
unsigned long elementEnd = 0;

enum CWState
{
  CW_IDLE,
  CW_DIT,
  CW_DAH,
  CW_GAP
};

CWState cwState = CW_IDLE;

// ================================================================
// FIFO
// ================================================================

#define FIFO_SIZE 128

char fifo[FIFO_SIZE];

volatile uint16_t fifoHead = 0;
volatile uint16_t fifoTail = 0;

bool fifoPush(char c)
{
  uint16_t next =
    (fifoHead + 1) % FIFO_SIZE;

  if (next == fifoTail)
    return false;

  fifo[fifoHead] = c;
  fifoHead = next;

  return true;
}

bool fifoAvailable()
{
  return fifoHead != fifoTail;
}

char fifoPop()
{
  if (!fifoAvailable())
    return 0;

  char c = fifo[fifoTail];

  fifoTail =
    (fifoTail + 1) % FIFO_SIZE;

  return c;
}

void fifoClear()
{
  fifoHead = 0;
  fifoTail = 0;
}

// ================================================================
// MORSE BUFFER
// ================================================================

char currentChar = 0;

const char *currentMorse = nullptr;

uint8_t morseIndex = 0;

bool sendingMorse = false;

// ================================================================
// USB SERIAL STABLE
// ================================================================

String getUSBSerial()
{
  uint8_t mac[6];

  esp_efuse_mac_get_default(mac);

  char serial[18];

  snprintf(
    serial,
    sizeof(serial),
    "%02X%02X%02X%02X%02X%02X",
    mac[0],
    mac[1],
    mac[2],
    mac[3],
    mac[4],
    mac[5]
  );

  return String(serial);
}

// ================================================================
// USB IDENTITE
// ================================================================

void setupUSBIdentity()
{
  String usbSerial = getUSBSerial();

  USB.VID(USB_VID);
  USB.PID(USB_PID);

  USB.manufacturerName(USB_MANUF);
  USB.productName(USB_PRODUCT);
  USB.serialNumber(usbSerial.c_str());

  USB.begin();
}

// ================================================================
// LED RMT
// ================================================================

void sendBit(rmt_item32_t *item, bool one)
{
  if (one)
  {
    item->level0 = 1;
    item->duration0 = WS_T1H;

    item->level1 = 0;
    item->duration1 = WS_T1L;
  }
  else
  {
    item->level0 = 1;
    item->duration0 = WS_T0H;

    item->level1 = 0;
    item->duration1 = WS_T0L;
  }
}

void sendLEDByte(
  rmt_item32_t *items,
  int &pos,
  uint8_t value)
{
  for (int i = 7; i >= 0; i--)
  {
    sendBit(
      &items[pos],
      value & (1 << i)
    );

    pos++;
  }
}

void setRGB(
  uint8_t r,
  uint8_t g,
  uint8_t b)
{
  rmt_item32_t items[25];

  int pos = 0;

  // WS2812 = GRB

  sendLEDByte(items, pos, g);
  sendLEDByte(items, pos, r);
  sendLEDByte(items, pos, b);

  // RESET

  items[pos].level0 = 0;
  items[pos].duration0 = WS_RESET;

  items[pos].level1 = 0;
  items[pos].duration1 = 0;

  pos++;

  rmt_write_items(
    LED_RMT_CHANNEL,
    items,
    pos,
    true
  );

  rmt_wait_tx_done(
    LED_RMT_CHANNEL,
    portMAX_DELAY
  );
}

void setupLED()
{
  rmt_config_t config = {};

  config.rmt_mode = RMT_MODE_TX;
  config.channel = LED_RMT_CHANNEL;
  config.gpio_num = (gpio_num_t)RGB_LED_PIN;

  config.clk_div = 2;
  config.mem_block_num = 1;

  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;

  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level =
    RMT_IDLE_LEVEL_LOW;

  rmt_config(&config);

  rmt_driver_install(
    LED_RMT_CHANNEL,
    0,
    0
  );
}

// ================================================================
// LED ETAT
// ================================================================

void ledReady()
{
  setRGB(0, 255, 0);
}

void ledTransmit()
{
  setRGB(255, 0, 0);
}

void ledWinKey()
{
  setRGB(0, 0, 255);
}

void ledWhite()
{
  setRGB(255, 255, 255);
}

void ledOff()
{
  setRGB(0, 0, 0);
}

// ================================================================
// SIDETONE
// ================================================================

void sidetoneStart()
{
  if (!sidetoneOn)
    return;

  ledTransmit();

  ledcWriteTone(
    0,
    SIDETONE_FREQ
  );
}

void sidetoneStop()
{
  ledReady();

  ledcWriteTone(
    0,
    0
  );
}

// ================================================================
// KEY OUTPUT
// ================================================================

void keyOn()
{
  if (!keyDown)
  {
    keyDown = true;

    digitalWrite(
      KEY_PIN,
      HIGH
    );

    sidetoneStart();
  }
}

void keyOff()
{
  if (keyDown)
  {
    keyDown = false;

    digitalWrite(
      KEY_PIN,
      LOW
    );

    sidetoneStop();
  }
}

// ================================================================
// WPM
// ================================================================

void updateTiming()
{
  // 1 dit = 1200 / WPM ms

  ditLength =
    1200UL / max(wpm, 1);

  dahLength =
    ditLength * 3;
}

void updatePot()
{
  static unsigned long lastPot = 0;

  if (millis() - lastPot < 100)
    return;

  lastPot = millis();

  int value =
    analogRead(SPEED_POT_PIN);

  int newWpm =
    map(
      value,
      0,
      4095,
      MIN_WPM,
      MAX_WPM
    );

  if (abs(newWpm - wpm) >= 1)
  {
    wpm = newWpm;

    updateTiming();
  }
}

// ================================================================
// PADDLES
// ================================================================

bool readDit()
{
  bool state =
    digitalRead(DIT_PIN) == LOW;

  return paddleReverse
    ? digitalRead(DAH_PIN) == LOW
    : state;
}

bool readDah()
{
  bool state =
    digitalRead(DAH_PIN) == LOW;

  return paddleReverse
    ? digitalRead(DIT_PIN) == LOW
    : state;
}

// ================================================================
// MORSE LOOKUP
// ================================================================

const char *findMorse(char c)
{
  c = toupper(c);

  for (int i = 0; i < MORSE_COUNT; i++)
  {
    if (morseTable[i].c == c)
      return morseTable[i].code;
  }

  return nullptr;
}

// ================================================================
// DEMARRAGE CARACTERE
// ================================================================

void startCharacter(char c)
{
  currentChar = toupper(c);

  currentMorse =
    findMorse(currentChar);

  morseIndex = 0;

  if (!currentMorse)
  {
    if (currentChar == ' ')
    {
      // espace inter-mot
      elementEnd =
        millis() + ditLength * 4;

      cwState = CW_GAP;
    }

    return;
  }

  sendingMorse = true;

  if (currentMorse[0] == '.')
  {
    keyOn();

    elementStart = millis();

    elementEnd =
      elementStart + ditLength;

    cwState = CW_DIT;
  }
  else
  {
    keyOn();

    elementStart = millis();

    elementEnd =
      elementStart + dahLength;

    cwState = CW_DAH;
  }
}

// ================================================================
// FIN ELEMENT
// ================================================================

void finishMorseElement()
{
  keyOff();

  morseIndex++;

  if (currentMorse[morseIndex] == '\0')
  {
    sendingMorse = false;

    // Fin caractère :
    // le temps de l'élément est déjà écoulé.
    // On ajoute 2 dit pour obtenir 3 au total.

    elementEnd =
      millis() + ditLength * 2;

    cwState = CW_GAP;

    return;
  }

  // séparation entre éléments
  elementEnd =
    millis() + ditLength;

  cwState = CW_GAP;
}

// ================================================================
// MOTEUR MORSE
// ================================================================

void processMorse()
{
  if (paused)
    return;

  unsigned long now = millis();

  // ------------------------------------------------------------
  // ETAT ACTUEL
  // ------------------------------------------------------------

  if (cwState == CW_DIT ||
      cwState == CW_DAH)
  {
    if (now >= elementEnd)
    {
      finishMorseElement();
    }

    return;
  }

  // ------------------------------------------------------------
  // GAP
  // ------------------------------------------------------------

  if (cwState == CW_GAP)
  {
    if (now < elementEnd)
      return;

    // Si le caractère est terminé
    if (!sendingMorse)
    {
      if (fifoAvailable())
      {
        char c = fifoPop();

        startCharacter(c);
      }
      else
      {
        cwState = CW_IDLE;
      }

      return;
    }

    // élément suivant
    if (currentMorse[morseIndex] == '.')
    {
      keyOn();

      elementStart = now;

      elementEnd =
        now + ditLength;

      cwState = CW_DIT;
    }
    else
    {
      keyOn();

      elementStart = now;

      elementEnd =
        now + dahLength;

      cwState = CW_DAH;
    }

    return;
  }

  // ------------------------------------------------------------
  // IDLE
  // ------------------------------------------------------------

  if (cwState == CW_IDLE)
  {
    if (fifoAvailable())
    {
      char c = fifoPop();

      startCharacter(c);
    }
  }
}

// ================================================================
// IAMBIC KEYER
// ================================================================

void processPaddles()
{
  if (paused)
    return;

  // Si le moteur Morse par FIFO est actif,
  // les paddles restent prioritaires seulement
  // lorsque le keyer est au repos.

  if (sendingMorse)
    return;

  static unsigned long paddleStart = 0;

  static bool paddleActive = false;

  bool dit = readDit();
  bool dah = readDah();

  if (cwState != CW_IDLE &&
      cwState != CW_GAP)
    return;

  if (!paddleActive)
  {
    if (dit)
    {
      paddleActive = true;

      keyOn();

      elementStart = millis();

      elementEnd =
        elementStart + ditLength;

      cwState = CW_DIT;

      ditMemory = false;
      dahMemory = false;

      return;
    }

    if (dah)
    {
      paddleActive = true;

      keyOn();

      elementStart = millis();

      elementEnd =
        elementStart + dahLength;

      cwState = CW_DAH;

      ditMemory = false;
      dahMemory = false;

      return;
    }
  }

  if (dit && cwState == CW_DAH)
    ditMemory = true;

  if (dah && cwState == CW_DIT)
    dahMemory = true;

  if (cwState == CW_DIT &&
      millis() >= elementEnd)
  {
    keyOff();

    // Iambic B : si l'autre paddle est mémorisé,
    // on envoie immédiatement l'élément opposé.

    if (iambicMode &&
        iambicType == IAMBIC_B &&
        dahMemory)
    {
      dahMemory = false;

      keyOn();

      elementStart = millis();

      elementEnd =
        elementStart + dahLength;

      cwState = CW_DAH;

      return;
    }

    if (dit)
    {
      keyOn();

      elementStart = millis();

      elementEnd =
        elementStart + ditLength;

      cwState = CW_DIT;

      return;
    }

    if (dah)
    {
      keyOn();

      elementStart = millis();

      elementEnd =
        elementStart + dahLength;

      cwState = CW_DAH;

      return;
    }

    cwState = CW_IDLE;
    paddleActive = false;

    return;
  }

  if (cwState == CW_DAH &&
      millis() >= elementEnd)
  {
    keyOff();

    if (iambicMode &&
        iambicType == IAMBIC_B &&
        ditMemory)
    {
      ditMemory = false;

      keyOn();

      elementStart = millis();

      elementEnd =
        elementStart + ditLength;

      cwState = CW_DIT;

      return;
    }

    if (dah)
    {
      keyOn();

      elementStart = millis();

      elementEnd =
        elementStart + dahLength;

      cwState = CW_DAH;

      return;
    }

    if (dit)
    {
      keyOn();

      elementStart = millis();

      elementEnd =
        elementStart + ditLength;

      cwState = CW_DIT;

      return;
    }

    cwState = CW_IDLE;
    paddleActive = false;
  }
}

// ================================================================
// WINKEY : REPONSES
// ================================================================

void wkSend(uint8_t value)
{
  Serial.write(value);
}

void wkSendStatus()
{
  uint8_t status = 0;

  if (keyDown)
    status |= 0x01;

  if (paused)
    status |= 0x02;

  if (fifoAvailable())
    status |= 0x04;

  wkSend(status);
}

// ================================================================
// WINKEY : COMMANDES
// ================================================================

void processWinKeyCommand(uint8_t cmd)
{
  switch (cmd)
  {
    // ------------------------------------------------------------
    // HOST OPEN
    // ------------------------------------------------------------

    case 0x00:
    {
      if (Serial.available())
      {
        uint8_t sub = Serial.read();

        if (sub == 0x02)
        {
          hostOpen = true;

          ledWinKey();

          // réponse version simplifiée
          wkSend(0x12);

          return;
        }

        if (sub == 0x04)
        {
          // Echo command
          wkSend(0x55);

          return;
        }

        if (sub == 0x00)
        {
          // Calibrate
          wkSend(0x00);

          return;
        }
      }

      break;
    }

    // ------------------------------------------------------------
    // SPEED
    // ------------------------------------------------------------

    case 0x02:
    {
      if (Serial.available())
      {
        uint8_t speed =
          Serial.read();

        if (speed >= MIN_WPM &&
            speed <= MAX_WPM)
        {
          wpm = speed;

          updateTiming();
        }
      }

      return;
    }

    // ------------------------------------------------------------
    // SIDETONE
    // ------------------------------------------------------------

    case 0x03:
    {
      if (Serial.available())
      {
        uint8_t tone =
          Serial.read();

        if (tone == 0)
          sidetoneOn = false;
        else
          sidetoneOn = true;
      }

      return;
    }

    // ------------------------------------------------------------
    // PADDLE REVERSE
    // ------------------------------------------------------------

    case 0x04:
    {
      if (Serial.available())
      {
        uint8_t value =
          Serial.read();

        paddleReverse =
          value != 0;
      }

      return;
    }

    // ------------------------------------------------------------
    // IAMBIC MODE
    // ------------------------------------------------------------

    case 0x05:
    {
      if (Serial.available())
      {
        uint8_t value =
          Serial.read();

        iambicMode = true;

        if (value == 0)
          iambicType = IAMBIC_A;
        else
          iambicType = IAMBIC_B;
      }

      return;
    }

    // ------------------------------------------------------------
    // PAUSE
    // ------------------------------------------------------------

    case 0x0E:
    {
      if (Serial.available())
      {
        uint8_t value =
          Serial.read();

        paused =
          value != 0;

        if (paused)
        {
          keyOff();
          fifoClear();
        }
      }

      return;
    }

    // ------------------------------------------------------------
    // STATUS
    // ------------------------------------------------------------

    case 0x0F:
    {
      wkSendStatus();

      return;
    }

    default:
      break;
  }
}

// ================================================================
// WINKEY RX
// ================================================================

void processSerial()
{
  static bool commandPending = false;

  static uint8_t command = 0;

  while (Serial.available())
  {
    uint8_t b =
      Serial.read();

    // ----------------------------------------------------------
    // Command byte
    // ----------------------------------------------------------

    if (!commandPending)
    {
      /*
         Dans WinKey, les caractères ASCII sont normalement
         envoyés directement pour être transformés en CW.

         Les commandes utilisent des valeurs de contrôle.
      */

      if (b < 0x20)
      {
        command = b;

        commandPending = true;

        continue;
      }

      // --------------------------------------------------------
      // ASCII
      // --------------------------------------------------------

      if (hostOpen)
      {
        if (b >= 32 &&
            b <= 126)
        {
          fifoPush((char)b);
        }
      }

      continue;
    }

    // ----------------------------------------------------------
    // Paramètre de commande
    // ----------------------------------------------------------

    processWinKeyCommand(command);

    commandPending = false;
  }
}

// ================================================================
// TEST DIRECT SERIAL
// ================================================================

void processLocalCommands()
{
  /*
     Commandes ASCII simples utiles pendant les essais.

     + = WPM +
     - = WPM -
     A = Iambic A
     B = Iambic B
     R = reverse
     P = pause
     S = reprise
     T = test sidetone
     ? = status
  */

  // Cette fonction est volontairement désactivée
  // lorsque WinKey est utilisé.
}

// ================================================================
// SETUP
// ================================================================

void setup()
{
  // --------------------------------------------------------------
  // GPIO
  // --------------------------------------------------------------

  pinMode(
    DIT_PIN,
    INPUT_PULLUP
  );

  pinMode(
    DAH_PIN,
    INPUT_PULLUP
  );

  pinMode(
    KEY_PIN,
    OUTPUT
  );

  digitalWrite(
    KEY_PIN,
    LOW
  );

  // --------------------------------------------------------------
  // ADC
  // --------------------------------------------------------------

  analogReadResolution(12);

  // --------------------------------------------------------------
  // SIDETONE LEDC
  // Arduino ESP32 Core 2.0.17
  // --------------------------------------------------------------

  ledcSetup(
    0,
    SIDETONE_FREQ,
    8
  );

  ledcAttachPin(
    SIDETONE_PIN,
    0
  );

  ledcWriteTone(
    0,
    0
  );

  // --------------------------------------------------------------
  // USB DEVICE
  // --------------------------------------------------------------

  setupUSBIdentity();

  // --------------------------------------------------------------
  // USB / WinKey
  // --------------------------------------------------------------

  Serial.begin(
    WK_BAUD,
    SERIAL_8N2
  );

  delay(500);

  // --------------------------------------------------------------
  // LED
  // --------------------------------------------------------------

  setupLED();

  ledWhite();

  delay(300);

  ledReady();

  // --------------------------------------------------------------
  // TIMING
  // --------------------------------------------------------------

  updateTiming();

  // --------------------------------------------------------------
  // Etat initial
  // --------------------------------------------------------------

  cwState = CW_IDLE;

  hostOpen = false;

  paused = false;

  fifoClear();
}

// ================================================================
// LOOP
// ================================================================

void loop()
{
  // 1. potentiomètre
  updatePot();

  // 2. réception USB / WinKey
  processSerial();

  // 3. moteur CW
  processMorse();

  // 4. paddle manuel
  processPaddles();

  // boucle rapide
}
