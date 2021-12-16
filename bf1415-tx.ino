#include <LiquidCrystal.h>
#include <EEPROM.h>

// Number of rows and columns on display
#define DISP_ROWS 4
#define DISP_COLS 20

// Rotary encoder pins
#define PIN_ENC_INT 2
#define PIN_ENC_DIR 3

// Switch pins
#define PIN_SW1 10
#define PIN_SW2 11
#define PIN_SW3 12
#define MAX_BUTTONS 3

// LED pin
#define PIN_LED 13

// Transmitter pins
#define PIN_TX_MUTE 14             // Mute audio
#define PIN_TX_DATA 15             // Data
#define PIN_TX_CK 16               // Clock
#define PIN_TX_CE 17               // Chip select

// Speaker pin
#define PIN_SPK 18

// ADC input
#define ADC_PWR 5

// Frequency limits - 65 to 130 MHz
#define TOP_TX_FREQ 1300
#define BOTTOM_TX_FREQ 650
#define HOME_FREQ 1006

// On-off time for flashing LED
#define FLASH_MS 50

#define EEPROM_VALID 0x55

// A button
typedef struct
{
    unsigned char pin;
    int pressTimer;
    void (*buttonFn)(void);
}  button_t;

// EEPROM data
typedef struct
{
    unsigned char valid;
    unsigned int txFreq;
    unsigned char txStereo;
} eeprom_t;

eeprom_t sysVars;
button_t buttons[MAX_BUTTONS];

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

volatile unsigned int newTxFreq;
int buttonId;

unsigned char muted;

long msCount;
int ledState;

void initDisplay()
{
    // Setup display
  
    lcd.begin(DISP_COLS, DISP_ROWS);
    lcd.setCursor(0, 0);
    lcd.print("Freq:");
    lcd.setCursor(12, 0);
    lcd.print("MHz");
    lcd.setCursor(0, 1);
    lcd.print("Mode:");
    lcd.setCursor(0, 3);
    lcd.print("<< FM Transmitter >>");
}

void initEncoder()
{
    // Setup rotary encoder
  
    pinMode(PIN_ENC_INT, INPUT);
    pinMode(PIN_ENC_DIR, INPUT);
    digitalWrite(PIN_ENC_INT, HIGH); // enable pull-ups
    digitalWrite(PIN_ENC_DIR, HIGH);
    attachInterrupt(0, encoderInterrupt, FALLING);
}

void initSwitches()
{
    // Setup switches
  
    int c;
    
    buttons[0].pin = PIN_SW1;
    buttons[0].buttonFn = modeChangeButton;
    buttons[1].pin = PIN_SW2;
    buttons[1].buttonFn = muteChangeButton;
    buttons[2].pin = PIN_SW3;
    buttons[2].buttonFn = homeFreqButton;
    for(c = 0; c < MAX_BUTTONS; c++)
    {
        pinMode(buttons[c].pin, INPUT);
        digitalWrite(buttons[c].pin, HIGH);  // enable pull-up
        buttons[c].pressTimer = 0;
    }
}

void initTx()
{
    // Setup transmitter controls and configure transmitter
  
    pinMode(PIN_TX_CE, OUTPUT);
    digitalWrite(PIN_TX_CE, LOW);
    pinMode(PIN_TX_CK, OUTPUT);
    digitalWrite(PIN_TX_CK, LOW);
    pinMode(PIN_TX_DATA, OUTPUT);
    digitalWrite(PIN_TX_DATA, LOW);
    pinMode(PIN_TX_MUTE, OUTPUT);
    digitalWrite(PIN_TX_MUTE, LOW);
    txChange();
    dispFreq();
    dispMode();
    dispMute();
}

void initLed()
{
    // Setup flashing LED
  
    msCount = millis();
    ledState = LOW;
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, ledState);
}    

void beep()
{
    tone(PIN_SPK, 1800, 75);
}

void txWrite(unsigned int txConfig)
{
    // Clock configuration value into transmitter
  
    int c;

    // See BF1415 datasheet
    
    digitalWrite(PIN_TX_CE, HIGH);
    delayMicroseconds(5);
    
    c = 16;
    while(c > 0)
    {
        if((txConfig & 0x0001)  == 0x0001)
        {
            digitalWrite(PIN_TX_DATA, HIGH);
        }
        else
        {
            digitalWrite(PIN_TX_DATA, LOW);
        }
        delayMicroseconds(5);
        
        digitalWrite(PIN_TX_CK, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_TX_CK, LOW);
        
        txConfig = (txConfig >> 1);
        
        c--;
    }
    
    delayMicroseconds(5);
    digitalWrite(PIN_TX_CE, LOW);
}

void txChange()
{
    // Change transmitter to new frequency
  
    unsigned int txConfig;
    
    sysVars.txFreq = newTxFreq;
    
    txConfig = sysVars.txFreq;
    
    if(sysVars.txStereo == true)
    {
        txConfig = txConfig | 0x0800;
    }

    // Rest of data word is always the same - from data sheet    
    txConfig = txConfig | 0x4000;
    
    txWrite(txConfig);
}

void dispFreq()
{
    // Display operating frequency
  
    float f;
    
    // Blank last displayed frequency
    lcd.setCursor(6, 0);
    lcd.print("     ");
    
    if(sysVars.txFreq < 1000)
    {
        lcd.setCursor(7, 0);
    }
    else
    {
        lcd.setCursor(6, 0);
    }
    
    // Display current frequency
    f = (float)(sysVars.txFreq) / 10.0;
    lcd.print(f, 1);

    Serial.print(f, 1);
    Serial.print("MHz\n");
}

void dispMode()
{
    // Display mono or stereo indicator
    
    lcd.setCursor(6, 1);
    if(sysVars.txStereo == true)
    {
        lcd.print("Stereo");
    }
    else
    {
        lcd.print("Mono  ");
    }
}

void dispMute()
{
    // Display mute indicator
  
    lcd.setCursor(14, 1);
    if(muted == false)
    {
        lcd.print("Unmute");
    }
    else
    {
        lcd.print("  Mute");
    }
}

void encoderInterrupt()
{
    // Rotary encoder interrupt handler
    
    // If encoder turned clockwise
    if(digitalRead(PIN_ENC_INT) != digitalRead(PIN_ENC_DIR))
    {
        // Increment frequency
        if(newTxFreq == TOP_TX_FREQ)
        {
            newTxFreq = BOTTOM_TX_FREQ;
        }
        else
        {
            newTxFreq++;
        }
    }
    else
    {
        // Decrement frequency
        if(newTxFreq == BOTTOM_TX_FREQ)
        {
            newTxFreq = TOP_TX_FREQ;
        }
        else
        {
            newTxFreq--;
        }
    }
}

void modeChangeButton()
{
    // Mono-stereo button press handler
    
    // Just toggle state each time button pressed
    if(sysVars.txStereo == false)
    {
        sysVars.txStereo = true;
    }
    else
    {
        sysVars.txStereo = false;
    }
    
    txChange();
    dispMode();
    saveConfig();
}

void muteChangeButton()
{
    // Mute button press handler
    
    // Just toggle state each time button is pressed
    if(muted == true)
    {
        muted = false;
        digitalWrite(PIN_TX_MUTE, LOW);
    }
    else
    {
        muted = true;
        digitalWrite(PIN_TX_MUTE, HIGH);
    }
    
    dispMute();
}

void homeFreqButton()
{
    // Home button press handler
    
    // Set frequency to default
    newTxFreq = HOME_FREQ;    
}

void defaultConfig()
{
    sysVars.valid = EEPROM_VALID;
    sysVars.txStereo = true;
    sysVars.txFreq = HOME_FREQ;
}
 
int getConfig()
{
    // Read system variables from internal EEPROM
    
    unsigned char *dPtr;
    int c;

    // Read sysVars from EEPROM
    dPtr = (unsigned char *)&sysVars;
    for(c = 0; c < sizeof(sysVars); c++)
    {
        *dPtr = EEPROM.read(c);
        dPtr++;
    }    
    
    // Default values if nothing there
    if(sysVars.valid != EEPROM_VALID)
    {
        defaultConfig(); 
      
        return false;
    }
    else
    {
        return true;
    }
}

void saveConfig()
{
    // Write system variables to internal EEPROM
    
    unsigned char *dPtr;
    int c;
    
    dPtr = (unsigned char *)&sysVars;
    for(c = 0; c < sizeof(sysVars); c++)
    {
        EEPROM.write(c, *dPtr);
        dPtr++;
    }
}

void flashLed()
{
    long now;
    
    now = millis();
    if(now - msCount > FLASH_MS)
    {
        msCount = now;
        if(ledState == LOW)
        {
            ledState = HIGH;
        }
        else
        {
            ledState = LOW;
        }
        
        digitalWrite(PIN_LED, ledState);
    }
}

void checkButtons()
{
    // See if a button is pressed and call handler if it has
    // Debounce switch action
  
    // If button is pressed
    if(digitalRead(buttons[buttonId].pin) == LOW)
    {
        // Wait 1ms
        delay(1);
        
        // If it's still pressed
        if(digitalRead(buttons[buttonId].pin) == LOW)
        {
            // Increment timer
            buttons[buttonId].pressTimer++;
            
            // If it's been pressed for 10ms
            if(buttons[buttonId].pressTimer == 10)
            {
                // Call handler function if there is one
                if(buttons[buttonId].buttonFn)
                {
                    beep();
                    buttons[buttonId].buttonFn();
                }
            }
        }
        else
        {
            // Not stable after 1ms, so reset timer
            buttons[buttonId].pressTimer = 0;
        }       
    }
    else
    {
        // Not pressed, so reset timer
        buttons[buttonId].pressTimer = 0;
    }

    // If button not pressed or not stable, then move on to next one   
    if(buttons[buttonId].pressTimer == 0)
    {
        buttonId++;
        if(buttonId == MAX_BUTTONS)
        {
            buttonId = 0;
        }
    }
}

void setup()
{
    // Initialisation
  
    Serial.begin(9600);
    Serial.print("Starting..\n");
    
    // Read EEPROM data
    if(getConfig() == false)
    {
        Serial.print("Using defaults for sysvars\n");
        saveConfig();
    }

    // Global variables
    buttonId = 0;
    newTxFreq = sysVars.txFreq;
    muted = false;
 
    // Display
    initDisplay();

    // Rotary encoder
    initEncoder();

    // Switches
    initSwitches();

    // Press home button for 500ms to reset everything
    if(digitalRead(buttons[2].pin) == LOW)
    {
         delay(500);
         if(digitalRead(buttons[2].pin) == LOW)
         {
             Serial.print("Reset to defaults\n");
             beep();
             defaultConfig();
         }
    }
    
    // LED
    initLed();

    // Transmitter controls
    initTx();

    Serial.print(analogRead(ADC_PWR));

    Serial.print("Ready.\n");
}

void loop()
{
    // Called periodically from main()
    
    // Change frequency if required
    // Check one push button each time through
    // Handle heartbeat LED
  
    // If the interrupt handler or the home button has changed the frequency
    if(newTxFreq != sysVars.txFreq)
    {
        // Change to new frequency
        txChange();

        // Display frequency
        dispFreq();
        
        // Save change in EEPROM
        saveConfig();
    }
 
    // Check push button   
    checkButtons();
       
    // Flash the heartbeat LED
    flashLed();
}
