# Command code manual:

## Motor control command format
    m[DIR][MOTOR][PWM]
    DIR -> f - forward, b - backward, r - right, l - left, s - stop, c - custom
    Motor -> 0 or 1
    PWM -> (0-255)


    
## Servo control command format
    s[servo_number][pos]
    pos -> (0-180)
    
## Custom control command format
    st
    [motor/servo command]
    [motor/servo command]
    .
    .
    .
    end

https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
link for arduino ide esp32 board installation