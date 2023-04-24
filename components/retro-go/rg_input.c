#include "rg_system.h"
#include "rg_input.h"

#include <stdlib.h>
#include <unistd.h>

#ifndef RG_TARGET_SDL2
#include <driver/gpio.h>
#else
#include <SDL2/SDL.h>
#endif

#if RG_GAMEPAD_DRIVER == 1 || defined(RG_BATTERY_ADC_CHANNEL)
#include <esp_adc_cal.h>
#include <driver/adc.h>
#define USE_ADC_DRIVER 1
#endif

static bool input_task_running = false;
static int64_t last_gamepad_read = 0;
static uint32_t gamepad_state = -1; // _Atomic
static int battery_level = -1;
#if USE_ADC_DRIVER
static esp_adc_cal_characteristics_t adc_chars;
#endif


#if RG_GAMEPAD_DRIVER == 7

static volatile int16_t EncoderDiff = 0;

#endif


static inline uint32_t gamepad_read(void)
{
    uint32_t state = 0;

#if RG_GAMEPAD_DRIVER == 1    // GPIO

    int joyX = adc1_get_raw(RG_GPIO_GAMEPAD_X);
    int joyY = adc1_get_raw(RG_GPIO_GAMEPAD_Y);

    if (!gpio_get_level(RG_GPIO_GAMEPAD_MENU))   state |= RG_KEY_MENU;
    if (!gpio_get_level(RG_GPIO_GAMEPAD_OPTION)) state |= RG_KEY_OPTION;
    if (!gpio_get_level(RG_GPIO_GAMEPAD_SELECT)) state |= RG_KEY_SELECT;
    if (!gpio_get_level(RG_GPIO_GAMEPAD_START))  state |= RG_KEY_START;
    if (!gpio_get_level(RG_GPIO_GAMEPAD_A))      state |= RG_KEY_A;
    if (!gpio_get_level(RG_GPIO_GAMEPAD_B))      state |= RG_KEY_B;

    #if RG_SCREEN_TYPE == 32
        if(joyY > 2048) state |= RG_KEY_UP;
        if(joyY > 1024 && joyY < 2048) state |= RG_KEY_DOWN;
        if(joyX > 2048) state |= RG_KEY_LEFT;
        if(joyX > 1024 && joyX < 2048) state |= RG_KEY_RIGHT;

        if (state == (RG_KEY_SELECT|RG_KEY_A))
            state = RG_KEY_OPTION;

        if (state == (RG_KEY_START|RG_KEY_SELECT))
            state = RG_KEY_MENU;
    #else
        if (joyY > 2048 + 1024) state |= RG_KEY_UP;
        else if (joyY > 1024)   state |= RG_KEY_DOWN;
        if (joyX > 2048 + 1024) state |= RG_KEY_LEFT;
        else if (joyX > 1024)   state |= RG_KEY_RIGHT;


    #endif
#elif RG_GAMEPAD_DRIVER == 7    // GPIO gamepad-self-encoder
    volatile int16_t diff;
    //int joyX = adc1_get_raw(RG_GPIO_GAMEPAD_X);
    //int joyY = adc1_get_raw(RG_GPIO_GAMEPAD_Y);

    if (gpio_get_level(RG_GPIO_GAMEPAD_MENU))   state |= RG_KEY_MENU;
    if (!gpio_get_level(RG_GPIO_GAMEPAD_OPTION)) state |= RG_KEY_OPTION;
    if (gpio_get_level(RG_GPIO_GAMEPAD_SELECT)) state |= RG_KEY_START;//RG_KEY_SELECT;   // pull down
    if (!gpio_get_level(RG_GPIO_GAMEPAD_START))  state |= RG_KEY_SELECT;// RG_KEY_START;
    if (gpio_get_level(RG_GPIO_GAMEPAD_A))      state |= RG_KEY_A;       // pull down
    if (gpio_get_level(RG_GPIO_GAMEPAD_B))      state |= RG_KEY_B;        // pull down
    if (!gpio_get_level(RG_GPIO_GAMEPAD_UP))     state |= RG_KEY_UP;
    if (!gpio_get_level(RG_GPIO_GAMEPAD_DOWN))     state |= RG_KEY_DOWN;
    if (!gpio_get_level(RG_GPIO_GAMEPAD_LEFT))     state |= RG_KEY_LEFT;
    if (!gpio_get_level(RG_GPIO_GAMEPAD_RIGHT))     state |= RG_KEY_RIGHT;

     diff = -EncoderDiff;
     if(diff)
     {
        // RG_LOGI("diff :%d \n\n",diff);
        if(diff>0)
        {
            // RG_LOGI("state |= RG_KEY_UP,before state: "PRINTF_BINARY_16"\n",PRINTF_BINVAL_16(state));
            state |= RG_KEY_RIGHT;
            EncoderDiff=0;
            // RG_LOGI("state |= RG_KEY_UP,after state: "PRINTF_BINARY_16"\n\n",PRINTF_BINVAL_16(state));
        }
        else if (diff<0)
        {
            // RG_LOGI("state |= RG_KEY_DOWN,before state: "PRINTF_BINARY_16"\n",PRINTF_BINVAL_16(state));
            state |= RG_KEY_LEFT;
            EncoderDiff=0;
            // RG_LOGI("state |= RG_KEY_DOWN,after state: "PRINTF_BINARY_16"\n\n",PRINTF_BINVAL_16(state));
        }
     }


    // #if RG_SCREEN_TYPE == 32
    //     if(joyY > 2048) state |= RG_KEY_UP;
    //     if(joyY > 1024 && joyY < 2048) state |= RG_KEY_DOWN;
    //     if(joyX > 2048) state |= RG_KEY_LEFT;
    //     if(joyX > 1024 && joyX < 2048) state |= RG_KEY_RIGHT;

    //     if (state == (RG_KEY_SELECT|RG_KEY_A))
    //         state = RG_KEY_OPTION;

    //     if (state == (RG_KEY_START|RG_KEY_SELECT))
    //         state = RG_KEY_MENU;
    // #else
    //     if (joyY > 2048 + 1024) state |= RG_KEY_UP;
    //     else if (joyY > 1024)   state |= RG_KEY_DOWN;
    //     if (joyX > 2048 + 1024) state |= RG_KEY_LEFT;
    //     else if (joyX > 1024)   state |= RG_KEY_RIGHT;
    // #endif

#elif RG_GAMEPAD_DRIVER == 2  // Serial
    gpio_set_level(RG_GPIO_GAMEPAD_LATCH, 0);
    usleep(5);
    gpio_set_level(RG_GPIO_GAMEPAD_LATCH, 1);
    usleep(1);

    for (int i = 0; i < 8; i++) {
	int pinValue = gpio_get_level(RG_GPIO_GAMEPAD_DATA);
	state |= pinValue << (7 - i);

	gpio_set_level(RG_GPIO_GAMEPAD_CLOCK, 0);
	usleep(1);
	gpio_set_level(RG_GPIO_GAMEPAD_CLOCK, 1);
	usleep(1);
    }

    #ifdef RG_GPIO_GAMEPAD_MENU
    if (!gpio_get_level(RG_GPIO_GAMEPAD_MENU))   state |= RG_KEY_MENU;
    #endif
    #ifdef RG_GPIO_GAMEPAD_OPTION
    if (!gpio_get_level(RG_GPIO_GAMEPAD_OPTION)) state |= RG_KEY_OPTION;
    #endif

#elif RG_GAMEPAD_DRIVER == 3  // I2C

    uint8_t data[5];
    if (rg_i2c_read(0x20, -1, &data, 5))
    {
        int buttons = ~((data[2] << 8) | data[1]);

        if (buttons & RG_GAMEPAD_MAP_MENU) state |= RG_KEY_MENU;
        if (buttons & RG_GAMEPAD_MAP_OPTION) state |= RG_KEY_OPTION;
        if (buttons & RG_GAMEPAD_MAP_START) state |= RG_KEY_START;
        if (buttons & RG_GAMEPAD_MAP_SELECT) state |= RG_KEY_SELECT;
        if (buttons & RG_GAMEPAD_MAP_UP) state |= RG_KEY_UP;
        if (buttons & RG_GAMEPAD_MAP_RIGHT) state |= RG_KEY_RIGHT;
        if (buttons & RG_GAMEPAD_MAP_DOWN) state |= RG_KEY_DOWN;
        if (buttons & RG_GAMEPAD_MAP_LEFT) state |= RG_KEY_LEFT;
        if (buttons & RG_GAMEPAD_MAP_A) state |= RG_KEY_A;
        if (buttons & RG_GAMEPAD_MAP_B) state |= RG_KEY_B;

        battery_level = data[4];
    }
    #ifdef RG_GPIO_GAMEPAD_MENU
        if (!gpio_get_level(RG_GPIO_GAMEPAD_MENU)) state |= RG_KEY_MENU;
    #endif
    #ifdef RG_GPIO_GAMEPAD_OPTION
	    if (!gpio_get_level(RG_GPIO_GAMEPAD_OPTION)) state |= RG_KEY_OPTION;
    #endif

#elif RG_GAMEPAD_DRIVER == 4  // I2C via AW9523

    uint16_t buttons = ~(rg_i2c_gpio_read_port(0) | rg_i2c_gpio_read_port(1) << 8);

    if (buttons & RG_GAMEPAD_MAP_MENU) state |= RG_KEY_MENU;
    if (buttons & RG_GAMEPAD_MAP_OPTION) state |= RG_KEY_OPTION;
    if (buttons & RG_GAMEPAD_MAP_START) state |= RG_KEY_START;
    if (buttons & RG_GAMEPAD_MAP_SELECT) state |= RG_KEY_SELECT;
    if (buttons & RG_GAMEPAD_MAP_UP) state |= RG_KEY_UP;
    if (buttons & RG_GAMEPAD_MAP_RIGHT) state |= RG_KEY_RIGHT;
    if (buttons & RG_GAMEPAD_MAP_DOWN) state |= RG_KEY_DOWN;
    if (buttons & RG_GAMEPAD_MAP_LEFT) state |= RG_KEY_LEFT;
    if (buttons & RG_GAMEPAD_MAP_A) state |= RG_KEY_A;
    if (buttons & RG_GAMEPAD_MAP_B) state |= RG_KEY_B;

    battery_level = 99;

#elif RG_GAMEPAD_DRIVER == 6

    const uint8_t *keys = SDL_GetKeyboardState(NULL);

    if (keys[RG_GAMEPAD_MAP_MENU]) state |= RG_KEY_MENU;
    if (keys[RG_GAMEPAD_MAP_OPTION]) state |= RG_KEY_OPTION;
    if (keys[RG_GAMEPAD_MAP_START]) state |= RG_KEY_START;
    if (keys[RG_GAMEPAD_MAP_SELECT]) state |= RG_KEY_SELECT;
    if (keys[RG_GAMEPAD_MAP_UP]) state |= RG_KEY_UP;
    if (keys[RG_GAMEPAD_MAP_LEFT]) state |= RG_KEY_LEFT;
    if (keys[RG_GAMEPAD_MAP_DOWN]) state |= RG_KEY_DOWN;
    if (keys[RG_GAMEPAD_MAP_RIGHT]) state |= RG_KEY_RIGHT;
    if (keys[RG_GAMEPAD_MAP_A]) state |= RG_KEY_A;
    if (keys[RG_GAMEPAD_MAP_B]) state |= RG_KEY_B;

#endif

    // Virtual buttons (combos) to replace essential missing buttons.
#if !RG_GAMEPAD_HAS_MENU_BTN
    state &= ~RG_KEY_MENU;
    if (state == (RG_KEY_START|RG_KEY_SELECT))
        state = RG_KEY_MENU;
#endif

    return state;
}

static void input_task(void *arg)
{
    const uint8_t debounce_level = 0x01; //0x03;
    uint8_t debounce[RG_KEY_COUNT] = {0};
    uint32_t local_gamepad_state = 0;

    // Discard the first read, it contains garbage in certain drivers
    gamepad_read();

    input_task_running = true;

    while (input_task_running)
    {
        uint32_t state = gamepad_read();
// #if RG_GAMEPAD_DRIVER == 7
//         RG_LOGI("[before] %d %d %d %d \n\n",debounce[0],debounce[1],debounce[2],debounce[3]);
// #endif
        for (int i = 0; i < RG_KEY_COUNT; ++i)
        {
            debounce[i] = ((debounce[i] << 1) | ((state >> i) & 1));
            debounce[i] &= debounce_level;

            if (debounce[i] == debounce_level) // Pressed
            {
                local_gamepad_state |= (1 << i);
            }
            else if (debounce[i] == 0x00) // Released
            {
                local_gamepad_state &= ~(1 << i);
            }
        }


        gamepad_state = local_gamepad_state;
// #if RG_GAMEPAD_DRIVER == 7
//         RG_LOGI("[after ] %d %d %d %d \n\n",debounce[0],debounce[1],debounce[2],debounce[3]);
//         RG_LOGI(" state :"PRINTF_BINARY_16"\n\n",PRINTF_BINVAL_16(state));
//         RG_LOGI("game pad state:"PRINTF_BINARY_16"\n\n",PRINTF_BINVAL_16(gamepad_state));
// #endif
        rg_task_delay(10);
    }

    input_task_running = false;
    gamepad_state = -1;
    rg_task_delete(NULL);
}

static void IRAM_ATTR rg_encoder_gpio_isr_handler(void* arg)
{
    static volatile int count, countLast;
    static volatile uint8_t a0, b0;
    static volatile uint8_t ab0;
    uint8_t a = gpio_get_level(RG_GPIO_GAMEPAD_ENCODER_A);
    uint8_t b = gpio_get_level(RG_GPIO_GAMEPAD_ENCODER_B);
    if (a != a0)
    {
        a0 = a;
        if (b != b0)
        {
            b0 = b;
            count += ((a == b) ? 1 : -1);
            if ((a == b) != ab0)
            {
                count += ((a == b) ? 1 : -1);
            }
            ab0 = (a == b);
        }
    }

    if (count != countLast)
    {
        EncoderDiff += (count - countLast) > 0 ? 1 : -1;
        countLast = count;
    }

}

void rg_input_init(void)
{
    if (input_task_running)
        return;

#if RG_GAMEPAD_DRIVER == 1  // GPIO

    const char *driver = "GPIO";

    adc1_config_width(ADC_WIDTH_MAX - 1);
    adc1_config_channel_atten(RG_GPIO_GAMEPAD_X, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(RG_GPIO_GAMEPAD_Y, ADC_ATTEN_DB_11);

    gpio_set_direction(RG_GPIO_GAMEPAD_MENU, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_MENU, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_OPTION, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(RG_GPIO_GAMEPAD_OPTION, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_SELECT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_SELECT, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_START, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(RG_GPIO_GAMEPAD_START, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_A, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_B, GPIO_PULLUP_ONLY);
#elif RG_GAMEPAD_DRIVER == 7  // GPIO gamepad self-encoder

    const char *driver = "GPIO-GAMEPAD";

    // adc1_config_width(ADC_WIDTH_MAX - 1);
    // adc1_config_channel_atten(RG_GPIO_GAMEPAD_X, ADC_ATTEN_DB_11);
    // adc1_config_channel_atten(RG_GPIO_GAMEPAD_Y, ADC_ATTEN_DB_11);

    gpio_set_direction(RG_GPIO_GAMEPAD_MENU, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_MENU, GPIO_PULLDOWN_ONLY);
    // gpio_set_pull_mode(RG_GPIO_GAMEPAD_MENU, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_OPTION, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_OPTION, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_SELECT, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(RG_GPIO_GAMEPAD_SELECT, GPIO_PULLUP_ONLY);

    gpio_set_direction(RG_GPIO_GAMEPAD_UP, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(RG_GPIO_GAMEPAD_UP, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_DOWN, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(RG_GPIO_GAMEPAD_DOWN, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_LEFT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_LEFT, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_RIGHT, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(RG_GPIO_GAMEPAD_RIGHT, GPIO_PULLUP_ONLY);

    gpio_set_direction(RG_GPIO_GAMEPAD_START, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_START, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_A, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(RG_GPIO_GAMEPAD_A, GPIO_PULLUP_ONLY);
    gpio_set_direction(RG_GPIO_GAMEPAD_B, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(RG_GPIO_GAMEPAD_B, GPIO_PULLUP_ONLY);


    RG_LOGI("gamepad encoder init \n\n");
    gpio_config_t gpio_encoder_A = {
        .pin_bit_mask = 1ULL << RG_GPIO_GAMEPAD_ENCODER_A,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        //.intr_type = GPIO_INTR_NEGEDGE,
        .pull_up_en = 1,
    };
    gpio_config(&gpio_encoder_A);

    gpio_set_direction(RG_GPIO_GAMEPAD_ENCODER_B,GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_ENCODER_B,GPIO_PULLUP_ONLY);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(RG_GPIO_GAMEPAD_ENCODER_A, rg_encoder_gpio_isr_handler, (void*)RG_GPIO_GAMEPAD_ENCODER_A);



#elif RG_GAMEPAD_DRIVER == 2  // Serial

    const char *driver = "SERIAL";

    gpio_set_direction(RG_GPIO_GAMEPAD_CLOCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(RG_GPIO_GAMEPAD_LATCH, GPIO_MODE_OUTPUT);
    gpio_set_direction(RG_GPIO_GAMEPAD_DATA, GPIO_MODE_INPUT);

    gpio_set_level(RG_GPIO_GAMEPAD_LATCH, 0);
    gpio_set_level(RG_GPIO_GAMEPAD_CLOCK, 1);

    #ifdef RG_GPIO_GAMEPAD_MENU
    gpio_set_direction(RG_GPIO_GAMEPAD_MENU, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RG_GPIO_GAMEPAD_MENU, GPIO_PULLUP_ONLY);
    #endif
    #ifdef RG_GPIO_GAMEPAD_OPTION
    gpio_set_direction(RG_GPIO_GAMEPAD_OPTION, GPIO_MODE_INPUT);
    #endif

#elif RG_GAMEPAD_DRIVER == 3  // I2C

    const char *driver = "I2C";

    rg_i2c_init();

    #ifdef RG_GPIO_GAMEPAD_MENU
    gpio_set_direction(RG_GPIO_GAMEPAD_MENU, GPIO_MODE_INPUT);
    #endif
    #ifdef RG_GPIO_GAMEPAD_OPTION
    gpio_set_direction(RG_GPIO_GAMEPAD_OPTION, GPIO_MODE_INPUT);
    #endif

#elif RG_GAMEPAD_DRIVER == 4  // I2C w/AW9523

    const char *driver = "AW9523-I2C";

    rg_i2c_gpio_init();

    // All that below should be moved elsewhere, and possibly specific to the qtpy...
    rg_i2c_gpio_set_direction(AW_TFT_RESET, 0);
    rg_i2c_gpio_set_direction(AW_TFT_BACKLIGHT, 0);
    rg_i2c_gpio_set_direction(AW_HEADPHONE_EN, 0);

    rg_i2c_gpio_set_level(AW_TFT_BACKLIGHT, 1);
    rg_i2c_gpio_set_level(AW_HEADPHONE_EN, 1);

    // tft reset
    rg_i2c_gpio_set_level(AW_TFT_RESET, 0);
    usleep(10 * 1000);
    rg_i2c_gpio_set_level(AW_TFT_RESET, 1);
    usleep(10 * 1000);

#elif RG_GAMEPAD_DRIVER == 6 // SDL2

	const char *driver = "SDL2";

#else

    #error "No gamepad driver selected"

#endif

#if USE_ADC_DRIVER
    adc1_config_width(ADC_WIDTH_MAX - 1);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_MAX - 1, 1100, &adc_chars);
#endif

    // Start background polling
    rg_task_create("rg_input", &input_task, NULL, 2 * 1024, RG_TASK_PRIORITY - 1, 1);
    while (gamepad_state == -1)
        rg_task_delay(1);
    RG_LOGI("Input ready. driver='%s',RG_GAMEPAD_DRIVER NUM %d, state=" PRINTF_BINARY_16 "\n\n", driver, RG_GAMEPAD_DRIVER ,PRINTF_BINVAL_16(gamepad_state));
#if RG_GAMEPAD_DRIVER == 7
    RG_LOGI("RG_GPIO_GAMEPAD_MENU   :%d \n",gpio_get_level(RG_GPIO_GAMEPAD_MENU));
    RG_LOGI("RG_GPIO_GAMEPAD_SELECT :%d \n",gpio_get_level(RG_GPIO_GAMEPAD_SELECT));
    RG_LOGI("RG_GPIO_GAMEPAD_A      :%d \n",gpio_get_level(RG_GPIO_GAMEPAD_A));
    RG_LOGI("RG_GPIO_GAMEPAD_UP     :%d \n\n",gpio_get_level(RG_GPIO_GAMEPAD_UP));
#endif

}

void rg_input_deinit(void)
{
    input_task_running = false;
    // while (gamepad_state != -1)
    //     rg_task_delay(1);
    RG_LOGI("Input terminated.\n");
}

long rg_input_gamepad_last_read(void)
{
    if (!last_gamepad_read)
        return 0;

    return rg_system_timer() - last_gamepad_read;
}

uint32_t rg_input_read_gamepad(void)
{
    last_gamepad_read = rg_system_timer();
#ifdef RG_TARGET_SDL2
    SDL_PumpEvents();
#endif
    return gamepad_state;
}

bool rg_input_key_is_pressed(rg_key_t key)
{
    return (rg_input_read_gamepad() & key) ? true : false;
}

void rg_input_wait_for_key(rg_key_t key, bool pressed)
{
    while (rg_input_key_is_pressed(key) != pressed)
        rg_task_delay(1);
}

bool rg_input_read_battery(float *percent, float *volts)
{
#if defined(RG_BATTERY_ADC_CHANNEL)
    uint32_t adc_sample = 0;

    for (int i = 0; i < 4; ++i)
    {
        adc_sample += esp_adc_cal_raw_to_voltage(adc1_get_raw(RG_BATTERY_ADC_CHANNEL), &adc_chars);
    }
    adc_sample /= 4;

    // We no longer do that because time between calls to read_battery can be significant.
    // If we really care we could average values on the caller side...
    // adc_sample += battery_level;
    // adc_sample /= 2;

    battery_level = adc_sample;
#else
    // We could read i2c here but the i2c API isn't threadsafe, so we'll rely on the input task.
#endif

    if (battery_level == -1) // No battery or error?
        return false;

    if (percent)
    {
        *percent = RG_BATTERY_CALC_PERCENT(battery_level);
        *percent = RG_MAX(0.f, RG_MIN(100.f, *percent));
    }

    if (volts)
        *volts = RG_BATTERY_CALC_VOLTAGE(battery_level);

    return true;
}
