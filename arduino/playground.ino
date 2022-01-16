#define LED_PIN 3
#define BUTTON_PIN 2
#define ENC_A_PIN 14
#define ENC_B_PIN 16

#define ENC_NONE -1
#define ENC_CW -2
#define ENC_CCW -3

#define ES_NEUTRAL 4

// 64 values
const uint8_t brightness_map[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 23, 24, 26, 27, 29, 31, 33, 35, 37, 39, 42, 44,
    47, 50, 53, 56, 60, 63, 67, 71, 76, 80, 85, 90, 95, 101, 107, 114,
    120, 128, 135, 143, 152, 161, 170, 180, 191, 203, 215, 227, 241, 255};

bool led = 1;
unsigned char brightness = 31;

// Input state tracking

int buttonState = HIGH;

int encoderState = ES_NEUTRAL;
int encoderHistory = 0xFF;

void setup()
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 0);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);
}

void loop()
{
    bool btn_pressed = readButton(digitalRead(BUTTON_PIN), &buttonState);
    int enc_direction = readEncoder(digitalRead(ENC_A_PIN), digitalRead(ENC_B_PIN), &encoderHistory, &encoderState);

    if (btn_pressed)
        led = !led;

    switch (enc_direction)
    {
    case ENC_CW:
        if (brightness < 63)
            brightness += 1;

        break;
    case ENC_CCW:
        if (brightness > 0)
            brightness -= 1;

        break;
    }

    if (led)
        analogWrite(LED_PIN, brightness_map[brightness]);
    else
        analogWrite(LED_PIN, 0);
}

bool readButton(int input, int *state)
{
    bool out = false;
    if (*state == HIGH && input == LOW)
    {
        out = true;
    }

    *state = input;
    return out;
}

// Rotary encoder handling

// next expected history pattern in each state
const int8_t expectedPattern[] = {
    0b1111, // neutral
    0b0111, // CCW D
    0b0001, // CCW C
    0b1000, // CCW B
    -1,     // 1101 (CW) or 1110 (CCW)
    0b0100, // CW B
    0b0010, // CW C
    0b1011, // CW D
    0b1111, // neutral
};
// next state (when history match successful)
const int8_t nextState[] = {ES_NEUTRAL, 0, 1, 2, -1, 6, 7, 8, ES_NEUTRAL};

int readEncoder(int a, int b, int *history, int *state)
{
    int signals = (a << 1) | b;                 // combine signals
    *history = (*history << 2 | signals) & 0xF; // update history

    if (*state == ES_NEUTRAL && *history == 0b1101)
        *state = 5; // CW A

    if (*state == ES_NEUTRAL && *history == 0b1110)
        *state = 3; // CCW A

    if (*history == expectedPattern[*state]) // this won't be true for state changed above
    {
        *state = nextState[*state];

        if (*state == 0) // CCW D
            return ENC_CCW;
        if (*state == 8) // CW D
            return ENC_CW;
    }

    // if both signals are high, reset to neutral state
    if (*history == 0xF)
        *state = ES_NEUTRAL;

    return ENC_NONE;
}
