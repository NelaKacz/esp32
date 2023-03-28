struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed; // the current state of button
};

struct Led {
    const uint8_t PIN;
    uint8_t state; // the current state of LED
};

Button button = {BUTTON_PIN, 0, false};
Led led = {LED_PIN, LOW};