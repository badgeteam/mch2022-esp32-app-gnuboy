/*
 * Copyright (c) 2022 Renze Nicolai
 * Copyright (c) 2019 Fuji Pebri
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <string.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_err.h>
#include <esp_log.h>
#include <soc/rtc.h>
#include <soc/rtc_cntl_reg.h>
#include <hardware.h>
#include <loader.h>
#include <hw.h>
#include <lcd.h>
#include <fb.h>
#include <cpu.h>
#include <mem.h>
#include <sound.h>
#include <pcm.h>
#include <regs.h>
#include <rtc_gnuboy.h>
#include <gnuboy.h>
#include <sdcard.h>
#include "audio.h"
#include "main.h"
#include "filesystems.h"
#include "ws2812.h"
#include "menu.h"
#include "graphics_wrapper.h"

#define SD_BASE_PATH "/sd"
#define GAMEBOY_WIDTH (160)
#define GAMEBOY_HEIGHT (144)
#define AUDIO_SAMPLE_RATE (32000)

#define AVERAGE(a, b) ( ((((a) ^ (b)) & 0xf7deU) >> 1) + ((a) & (b)) )
#define LINE_BUFFERS (4)
#define LINE_COUNT   (19)
#define LINE_COUNT_UNSCALED   (4)

static const char *TAG = "main";

extern const uint8_t border_png_start[] asm("_binary_border_png_start");
extern const uint8_t border_png_end[] asm("_binary_border_png_end");

extern const uint8_t load_png_start[] asm("_binary_load_png_start");
extern const uint8_t load_png_end[] asm("_binary_load_png_end");

extern const uint8_t save_png_start[] asm("_binary_save_png_start");
extern const uint8_t save_png_end[] asm("_binary_save_png_end");

extern const uint8_t rom_png_start[] asm("_binary_rom_png_start");
extern const uint8_t rom_png_end[] asm("_binary_rom_png_end");

extern const uint8_t state_load_png_start[] asm("_binary_state_load_png_start");
extern const uint8_t state_load_png_end[] asm("_binary_state_load_png_end");

extern const uint8_t state_save_png_start[] asm("_binary_state_save_png_start");
extern const uint8_t state_save_png_end[] asm("_binary_state_save_png_end");

extern const uint8_t play_png_start[] asm("_binary_play_png_start");
extern const uint8_t play_png_end[] asm("_binary_play_png_end");

extern const uint8_t joystick_png_start[] asm("_binary_joystick_png_start");
extern const uint8_t joystick_png_end[] asm("_binary_joystick_png_end");

extern const uint8_t volume_up_png_start[] asm("_binary_volume_up_png_start");
extern const uint8_t volume_up_png_end[] asm("_binary_volume_up_png_end");

extern const uint8_t volume_down_png_start[] asm("_binary_volume_down_png_start");
extern const uint8_t volume_down_png_end[] asm("_binary_volume_down_png_end");

extern void cpu_reset();
extern int cpu_emulate(int cycles);
extern void loadstate_rom(const unsigned char* rom);

struct fb fb;
struct pcm pcm;

static nvs_handle_t nvs_handle_gnuboy;

char rom_filename[512] = {0};

uint16_t* displayBuffer[2]; //= { fb0, fb0 }; //[160 * 144];
uint8_t currentBuffer = 0;

int16_t* audioBuffer[2];
volatile uint8_t currentAudioBuffer = 0;
volatile uint16_t currentAudioSampleCount;
volatile int16_t* currentAudioBufferPtr;

int frame = 0;
uint elapsedTime = 0;

QueueHandle_t vidQueue;
QueueHandle_t audioQueue;

ILI9341* ili9341 = NULL;
static pax_buf_t pax_buffer;
xQueueHandle button_queue;
uint16_t* framebuffer = NULL;

uint16_t* line[LINE_BUFFERS];

const pax_font_t *font = pax_font_saira_condensed;

uint8_t* rom_data = NULL;

pax_buf_t border;

// Note: Magic number obtained by adjusting until audio buffer overflows stop.
const int audioBufferLength = AUDIO_SAMPLE_RATE / 10 + 1;
const int AUDIO_BUFFER_SIZE = audioBufferLength * sizeof(int16_t) * 2;

esp_err_t nvs_get_str_fixed(nvs_handle_t handle, const char* key, char* target, size_t target_size, size_t* size) {
    esp_err_t    res;

    size_t required_size;
    res = nvs_get_str(handle, key, NULL, &required_size);
    if (res != ESP_OK) {
        return res;
    }

    if (required_size > target_size) {
        return ESP_FAIL;
    }

    res = nvs_get_str(handle, key, target, &required_size);

    if (size != NULL) *size = required_size;

    return res;
}

bool wait_for_button() {
    while (1) {
        rp2040_input_message_t message = {0};
        if (xQueueReceive(button_queue, &message, portMAX_DELAY) == pdTRUE) {
            if (message.state) {
                switch (message.input) {
                    case RP2040_INPUT_BUTTON_BACK:
                    case RP2040_INPUT_BUTTON_HOME:
                        return false;
                    case RP2040_INPUT_BUTTON_ACCEPT:
                        return true;
                    default:
                        break;
                }
            }
        }
    }
}

void disp_flush() {
    ili9341_write(get_ili9341(), pax_buffer.buf);
}

void exit_to_launcher() {
    nvs_close(nvs_handle_gnuboy);
    REG_WRITE(RTC_CNTL_STORE0_REG, 0);
    esp_restart();
}

void display_state(const char* text, uint16_t delay) {
    pax_draw_image(&pax_buffer, &border, 0, 0);
    pax_draw_text(&pax_buffer, 0xFFFFFFFF, font, 18, 0, pax_buffer.height - 18, text);
    disp_flush();
    vTaskDelay(pdMS_TO_TICKS(delay));
}

void display_fatal_error(const char* line0, const char* line1, const char* line2, const char* line3) {
    const pax_font_t* font = pax_font_saira_regular;
    pax_noclip(&pax_buffer);
    pax_background(&pax_buffer, 0xa85a32);
    if (line0 != NULL) pax_draw_text(&pax_buffer, 0xFFFFFFFF, font, 23, 0, 20 * 0, line0);
    if (line1 != NULL) pax_draw_text(&pax_buffer, 0xFFFFFFFF, font, 18, 0, 20 * 1, line1);
    if (line2 != NULL) pax_draw_text(&pax_buffer, 0xFFFFFFFF, font, 18, 0, 20 * 2, line2);
    if (line3 != NULL) pax_draw_text(&pax_buffer, 0xFFFFFFFF, font, 18, 0, 20 * 3, line3);
    disp_flush();
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void show_error(const char* message, uint16_t delay) {
    ESP_LOGE(TAG, "%s", message);
    pax_background(&pax_buffer, 0xa85a32);
    render_message(&pax_buffer, message);
    disp_flush();
    vTaskDelay(pdMS_TO_TICKS(delay));
}

void show_message(const char* message, uint16_t delay) {
    ESP_LOGI(TAG, "%s", message);
    pax_background(&pax_buffer, 0xFFFFFF);
    render_message(&pax_buffer, message);
    disp_flush();
    vTaskDelay(pdMS_TO_TICKS(delay));
}

static uint16_t getPixel(const uint16_t * bufs, int x, int y, int w1, int h1, int w2, int h2)
{
    int x_diff, y_diff, xv, yv, red , green, blue, col, a, b, c, d, index;
    int x_ratio = (int) (((w1-1)<<16)/w2) + 1;
    int y_ratio = (int) (((h1-1)<<16)/h2) + 1;

    xv = (int) ((x_ratio * x)>>16);
    yv = (int) ((y_ratio * y)>>16);

    x_diff = ((x_ratio * x)>>16) - (xv);
    y_diff = ((y_ratio * y)>>16) - (yv);

    index = yv*w1+xv ;

    a = bufs[index];
    b = bufs[index+1];
    c = bufs[index+w1];
    d = bufs[index+w1+1];

    red = (((a >> 11) & 0x1f) * (1-x_diff) * (1-y_diff) + ((b >> 11) & 0x1f) * (x_diff) * (1-y_diff) +
        ((c >> 11) & 0x1f) * (y_diff) * (1-x_diff) + ((d >> 11) & 0x1f) * (x_diff * y_diff));

    green = (((a >> 5) & 0x3f) * (1-x_diff) * (1-y_diff) + ((b >> 5) & 0x3f) * (x_diff) * (1-y_diff) +
        ((c >> 5) & 0x3f) * (y_diff) * (1-x_diff) + ((d >> 5) & 0x3f) * (x_diff * y_diff));

    blue = (((a) & 0x1f) * (1-x_diff) * (1-y_diff) + ((b) & 0x1f) * (x_diff) * (1-y_diff) +
        ((c) & 0x1f) * (y_diff) * (1-x_diff) + ((d) & 0x1f) * (x_diff * y_diff));

    col = ((int)red << 11) | ((int)green << 5) | ((int)blue);

    return col;
}

void write_gb_frame(const uint16_t * data) {
    short x,y;
    int sending_line=-1;
    int calc_line=0;
    
    if (data == NULL) return;
    
    bool scaling = true;
    
    if (scaling) {            
        int outputHeight = ILI9341_HEIGHT - 50;
        int outputWidth = GAMEBOY_WIDTH + (ILI9341_HEIGHT - 50 - GAMEBOY_HEIGHT);
        int xpos = (ILI9341_WIDTH - outputWidth) / 2;
        for (y=0; y<outputHeight; y+=LINE_COUNT)  {
            for (int i = 0; i < LINE_COUNT; ++i)
            {
                if((y + i) >= outputHeight) break;

                int index = (i) * outputWidth;
            
                for (x=0; x<outputWidth; ++x) 
                {
                    
                    uint16_t sample = getPixel(data, x, (y+i), GAMEBOY_WIDTH, GAMEBOY_HEIGHT, outputWidth, outputHeight);
                    line[calc_line][index]=((sample >> 8) | ((sample) << 8));
                    index++;
                }
            }
            sending_line=calc_line;
            calc_line = (calc_line + 1) % LINE_BUFFERS;
            ili9341_write_partial_direct(ili9341, (uint8_t*) line[sending_line], xpos, y + 25, outputWidth, LINE_COUNT);
        }
    }
    else
    {
        int ypos = (ILI9341_HEIGHT - GAMEBOY_HEIGHT)/2;
        int xpos = (ILI9341_WIDTH - GAMEBOY_WIDTH)/2;

        for (y=0; y<GAMEBOY_HEIGHT; y+=LINE_COUNT_UNSCALED)
        {
            for (int i = 0; i < LINE_COUNT_UNSCALED; ++i)
            {
                if((y + i) >= GAMEBOY_HEIGHT) break;

                int index = (i) * GAMEBOY_WIDTH;
                int bufferIndex = ((y + i) * GAMEBOY_WIDTH);

                for (x = 0; x < GAMEBOY_WIDTH; ++x)
                {
                    uint16_t sample = data[bufferIndex++];
                    line[calc_line][index++] = ((sample >> 8) | ((sample & 0xff) << 8));
                }
            }
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            ili9341_write_partial_direct(ili9341, (uint8_t*) line[sending_line], xpos, y+ypos, GAMEBOY_WIDTH, LINE_COUNT_UNSCALED);
        }
    }
}

volatile bool videoTaskIsRunning = false;
void videoTask(void *arg) {
  videoTaskIsRunning = true;
  uint16_t* param;
  while(1) {
        xQueuePeek(vidQueue, &param, portMAX_DELAY);
        if (param == (uint16_t*) 1) break;
        write_gb_frame(param);
        xQueueReceive(vidQueue, &param, portMAX_DELAY);
    }
    videoTaskIsRunning = false;
    vTaskDelete(NULL);
}

void run_to_vblank() {
  /* FRAME BEGIN */

  /* FIXME: djudging by the time specified this was intended
  to emulate through vblank phase which is handled at the
  end of the loop. */
  cpu_emulate(2280);

  /* FIXME: R_LY >= 0; comparsion to zero can also be removed
  altogether, R_LY is always 0 at this point */
  while (R_LY > 0 && R_LY < 144)
  {
    /* Step through visible line scanning phase */
    emu_step();
  }

  /* VBLANK BEGIN */

  if ((frame % 2) == 0) {
      xQueueSend(vidQueue, &framebuffer, portMAX_DELAY);
      // swap buffers
      currentBuffer = currentBuffer ? 0 : 1;
      framebuffer = displayBuffer[currentBuffer];
      fb.ptr = (uint8_t*) framebuffer;
  }
  rtc_tick();

  sound_mix();

  if (pcm.pos > 100)
  {
        currentAudioBufferPtr = (int16_t*) audioBuffer[currentAudioBuffer];
        currentAudioSampleCount = pcm.pos;

        void* tempPtr = (void*) 0x1234;
        xQueueSend(audioQueue, &tempPtr, portMAX_DELAY);

        // Swap buffers
        currentAudioBuffer = currentAudioBuffer ? 0 : 1;
        pcm.buf = audioBuffer[currentAudioBuffer];
        pcm.pos = 0;
  }

  if (!(R_LCDC & 0x80)) {
    /* LCDC operation stopped */
    /* FIXME: djudging by the time specified, this is
    intended to emulate through visible line scanning
    phase, even though we are already at vblank here */
    cpu_emulate(32832);
  }

  while (R_LY > 0) {
    /* Step through vblank phase */
    emu_step();
  }
}

char* system_util_GetFileName(const char* path)
{
    int length = strlen(path);
    int fileNameStart = length;

    if (fileNameStart < 1) abort();

    while (fileNameStart > 0)
    {
        if (path[fileNameStart] == '/')
        {
            ++fileNameStart;
            break;
        }

        --fileNameStart;
    }

    int size = length - fileNameStart + 1;

    char* result = malloc(size);
    if (!result) abort();

    result[size - 1] = 0;
    for (int i = 0; i < size - 1; ++i)
    {
        result[i] = path[fileNameStart + i];
    }

    printf("GetFileName: result='%s'\n", result);

    return result;
}

char* sdcard_create_savefile_path(const char* base_path, const char* fileName, const char* ext)
{
    char* result = NULL;

    if (!base_path) abort();
    if (!fileName) abort();

    printf("%s: base_path='%s', fileName='%s'\n", __func__, base_path, fileName);

    // Determine folder
    const char* extension = fileName + strlen(fileName); // place at NULL terminator
    while (extension != fileName)
    {
        if (*extension == '.')
        {
            ++extension;
            break;
        }
        --extension;
    }

    if (extension == fileName)
    {
        printf("%s: File extention not found.\n", __func__);
        abort();
    }

    //printf("%s: extension='%s'\n", __func__, extension);

    const char* DATA_PATH = "/gnuboy/";

    size_t savePathLength = strlen(base_path) + strlen(DATA_PATH) + strlen(fileName) + strlen(ext) + 1;
    char* savePath = malloc(savePathLength);
    if (savePath)
    {
        strcpy(savePath, base_path);
        strcat(savePath, DATA_PATH);
        strcat(savePath, fileName);
        strcat(savePath, ext);

        printf("%s: savefile_path='%s'\n", __func__, savePath);

        result = savePath;
    }

    return result;
}

static bool isOpen = false;

size_t sdcard_copy_file_to_memory(const char* path, void* ptr)
{
    size_t ret = 0;

    if (!isOpen)
    {
        printf("sdcard_copy_file_to_memory: not open.\n");
    }
    else
    {
        if (!ptr)
        {
            printf("sdcard_copy_file_to_memory: ptr is null.\n");
        }
        else
        {
            FILE* f = fopen(path, "rb");
            if (f == NULL)
            {
                printf("sdcard_copy_file_to_memory: fopen failed.\n");
            }
            else
            {
                // copy
                const size_t BLOCK_SIZE = 512;
                while(true)
                {
                    __asm__("memw");
                    size_t count = fread((uint8_t*)ptr + ret, 1, BLOCK_SIZE, f);
                    __asm__("memw");

                    ret += count;

                    if (count < BLOCK_SIZE) break;
                }
            }
        }
    }

    return ret;
}

void load_state() {
    if (rom_filename[0] == '\0') {
        show_error("No ROM loaded", 100);
        return;
    }
    char* fileName = system_util_GetFileName(rom_filename);
    if (!fileName) abort();
    char* pathName = sdcard_create_savefile_path(SD_BASE_PATH, fileName, ".sta");
    if (!pathName) abort();
    FILE* f = fopen(pathName, "r");
    if (f == NULL) {
        printf("load_state: fopen load failed\n");
    } else {
        loadstate(f);
        fclose(f);
        vram_dirty();
        pal_dirty();
        sound_dirty();
        mem_updatemap();
        printf("load_state: loadstate OK.\n");
    }
    free(pathName);
    free(fileName);
    display_state("State loaded", 50);
}

void save_state() {
    if (rom_filename[0] == '\0') {
        show_error("No ROM loaded", 100);
        return;
    }
    char* fileName = system_util_GetFileName(rom_filename);
    if (!fileName) abort();
    char* pathName = sdcard_create_savefile_path(SD_BASE_PATH, fileName, ".sta");
    if (!pathName) abort();
    FILE* f = fopen(pathName, "w");
    if (f == NULL) {
        printf("%s: fopen save failed (%s)\n", __func__, pathName);
        free(pathName);
        free(fileName);
        return;
    }
    savestate(f);
    fclose(f);
    printf("%s: savestate OK.\n", __func__);
    free(pathName);
    free(fileName);
    show_message("Game state saved", 50);
}

void load_sram() {
    if (rom_filename[0] == '\0') {
        show_error("No ROM loaded", 100);
        return;
    }
    char* fileName = system_util_GetFileName(rom_filename);
    if (!fileName) abort();
    char* pathName = sdcard_create_savefile_path(SD_BASE_PATH, fileName, ".srm");
    if (!pathName) abort();
    FILE* f = fopen(pathName, "r");
    if (f == NULL) {
        display_state("Failed to load SRAM", 100);
    } else {
        sram_load(f);
        fclose(f);
        vram_dirty();
        pal_dirty();
        sound_dirty();
        mem_updatemap();
        printf("SRAM loaded.\n");
        display_state("SRAM loaded", 50);
    }
    free(pathName);
    free(fileName);
}

void save_sram() {
    if (rom_filename[0] == '\0') {
        show_error("No ROM loaded", 100);
        return;
    }
    char* fileName = system_util_GetFileName(rom_filename);
    if (!fileName) abort();
    char* pathName = sdcard_create_savefile_path(SD_BASE_PATH, fileName, ".srm");
    if (!pathName) abort();
    FILE* f = fopen(pathName, "w");
    if (f == NULL) {
        printf("SRAM save failed\n");
        free(pathName);
        free(fileName);
        return;
    }
    sram_save(f);
    fclose(f);
    printf("SRAM saved.");
    free(pathName);
    free(fileName);
    display_state("SRAM saved", 100);
}


int pcm_submit() {
    audio_submit(currentAudioBufferPtr, currentAudioSampleCount >> 1);
    return 1;
}

volatile bool AudioTaskIsRunning = false;
void audioTask(void* arg) {
  uint16_t* param;

  AudioTaskIsRunning = true;
  while(1) {
    xQueuePeek(audioQueue, &param, portMAX_DELAY);
    if (param == 0) {
        // TODO: determine if this is still needed
        abort();
    } else if (param == 1) {
        break;
    } else {
        pcm_submit();
    }
    xQueueReceive(audioQueue, &param, portMAX_DELAY);
  }

  printf("audioTask: exiting.\n");

  AudioTaskIsRunning = false;
  vTaskDelete(NULL);

  while (1) {}
}

uint8_t* load_file_to_ram(FILE* fd, size_t* fsize) {
    fseek(fd, 0, SEEK_END);
    *fsize = ftell(fd);
    fseek(fd, 0, SEEK_SET);
    uint8_t* file = heap_caps_malloc(*fsize, MALLOC_CAP_SPIRAM);
    if (file == NULL) return NULL;
    fread(file, *fsize, 1, fd);
    return file;
}

typedef enum action {
    ACTION_NONE,
    ACTION_LOAD_ROM,
    ACTION_LOAD_ROM_INT,
    ACTION_LOAD_SAVE,
    ACTION_STORE_SAVE,
    ACTION_LOAD_STATE,
    ACTION_STORE_STATE,
    ACTION_RUN,
    ACTION_EXIT,
    ACTION_VOLUME_DOWN,
    ACTION_VOLUME_UP,
    ACTION_RESET
} menu_action_t;

static size_t menu_pos = 0;

menu_action_t show_menu() {
    menu_t* menu = menu_alloc("GNUBOY Gameboy emulator", 34, 18);
    menu->fgColor           = 0xFF000000;
    menu->bgColor           = 0xFFFFFFFF;
    menu->bgTextColor       = 0xFF000000;
    menu->selectedItemColor = 0xFFfec859;
    menu->borderColor       = 0xFF491d88;
    menu->titleColor        = 0xFF491d88;
    menu->titleBgColor      = 0xFFfec859;
    menu->scrollbarBgColor  = 0xFFCCCCCC;
    menu->scrollbarFgColor  = 0xFF555555;
    menu->grid_entry_count_y = 3;

    pax_buf_t icon_rom;
    pax_decode_png_buf(&icon_rom, (void*) rom_png_start, rom_png_end - rom_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_save;
    pax_decode_png_buf(&icon_save, (void*) save_png_start, save_png_end - save_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_load;
    pax_decode_png_buf(&icon_load, (void*) load_png_start, load_png_end - load_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_play;
    pax_decode_png_buf(&icon_play, (void*) play_png_start, play_png_end - play_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_state_save;
    pax_decode_png_buf(&icon_state_save, (void*) state_save_png_start, state_save_png_end - state_save_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_state_load;
    pax_decode_png_buf(&icon_state_load, (void*) state_load_png_start, state_load_png_end - state_load_png_start, PAX_BUF_32_8888ARGB, 0);    
    pax_buf_t icon_joystick;
    pax_decode_png_buf(&icon_joystick, (void*) joystick_png_start, joystick_png_end - joystick_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_volume_up;
    pax_decode_png_buf(&icon_volume_up, (void*) volume_up_png_start, volume_up_png_end - volume_up_png_start, PAX_BUF_32_8888ARGB, 0);
    pax_buf_t icon_volume_down;
    pax_decode_png_buf(&icon_volume_down, (void*) volume_down_png_start, volume_down_png_end - volume_down_png_start, PAX_BUF_32_8888ARGB, 0);
    
    menu_set_icon(menu, &icon_joystick);
    
    menu_insert_item_icon(menu, "Browse SD", NULL, (void*) ACTION_LOAD_ROM, -1, &icon_rom);
    menu_insert_item_icon(menu, "Browse int.", NULL, (void*) ACTION_LOAD_ROM_INT, -1, &icon_rom);
    menu_insert_item(menu, "", NULL, (void*) ACTION_NONE, -1);
    menu_insert_item_icon(menu, "Play!", NULL, (void*) ACTION_RUN, -1, &icon_play);
    menu_insert_item_icon(menu, "Vol. down", NULL, (void*) ACTION_VOLUME_DOWN, -1, &icon_volume_down);
    menu_insert_item_icon(menu, "Vol. up", NULL, (void*) ACTION_VOLUME_UP, -1, &icon_volume_up);
    menu_insert_item_icon(menu, "Load state", NULL, (void*) ACTION_LOAD_STATE, -1, &icon_state_load);
    menu_insert_item_icon(menu, "Save state", NULL, (void*) ACTION_STORE_STATE, -1, &icon_state_save);
    menu_insert_item_icon(menu, "Reset state", NULL, (void*) ACTION_RESET, -1, &icon_play);
    //menu_insert_item_icon(menu, "Load RAM", NULL, (void*) ACTION_LOAD_SAVE, -1, &icon_load);
    //menu_insert_item_icon(menu, "Save RAM", NULL, (void*) ACTION_STORE_SAVE, -1, &icon_save);
    
    bool render = true;
    bool quit = false;
    menu_action_t action = ACTION_NONE;
    pax_noclip(&pax_buffer);
    
    menu_set_position(menu, menu_pos);
    
    while (!quit) {
        if (render) {
            const pax_font_t* font = pax_font_saira_regular;
            pax_background(&pax_buffer, 0xFFFFFF);
            menu_render_grid(&pax_buffer, menu, 0, 0, 320, 220);//160);
            pax_draw_text(&pax_buffer, 0xFF491d88, font, 18, 5, 240 - 18, "🅰 select 🅷 exit");
            disp_flush();
            render = false;
        }
        rp2040_input_message_t message = {0};
        if (xQueueReceive(button_queue, &message, portMAX_DELAY) == pdTRUE) {
            if (message.state) {
                switch (message.input) {
                    case RP2040_INPUT_JOYSTICK_DOWN:
                        menu_navigate_next_row(menu);
                        render = true;
                        break;
                    case RP2040_INPUT_JOYSTICK_UP:
                        menu_navigate_previous_row(menu);
                        render = true;
                        break;
                    case RP2040_INPUT_JOYSTICK_LEFT:
                        menu_navigate_previous(menu);
                        render = true;
                        break;
                    case RP2040_INPUT_JOYSTICK_RIGHT:
                        menu_navigate_next(menu);
                        render = true;
                        break;
                    case RP2040_INPUT_BUTTON_HOME:
                        action = ACTION_EXIT;
                        quit = true;
                        break;
                    case RP2040_INPUT_BUTTON_ACCEPT:
                    case RP2040_INPUT_JOYSTICK_PRESS:
                    case RP2040_INPUT_BUTTON_SELECT:
                    case RP2040_INPUT_BUTTON_START:
                        action = (menu_action_t) menu_get_callback_args(menu, menu_get_position(menu));
                        quit = true;
                        break;
                    case RP2040_INPUT_BUTTON_MENU:
                        action = ACTION_RUN;
                        quit = true;
                        break;
                    default:
                        break;
                }
            }
        }
    }
    
    menu_pos = menu_get_position(menu);
    
    menu_free(menu);
    pax_buf_destroy(&icon_joystick);
    pax_buf_destroy(&icon_load);
    pax_buf_destroy(&icon_play);
    pax_buf_destroy(&icon_rom);
    pax_buf_destroy(&icon_save);
    pax_buf_destroy(&icon_state_load);
    pax_buf_destroy(&icon_state_save);
    pax_buf_destroy(&icon_volume_up);
    pax_buf_destroy(&icon_volume_down);
    return action;
}

void reset_and_init() {
    emu_reset();

    //&rtc.carry, &rtc.stop,
    rtc.d = 1;
    rtc.h = 1;
    rtc.m = 1;
    rtc.s = 1;
    rtc.t = 1;

    // vid_begin
    memset(&fb, 0, sizeof(fb));
    fb.w = 160;
    fb.h = 144;
    fb.pelsize = 2;
    fb.pitch = fb.w * fb.pelsize;
    fb.indexed = 0;
    fb.ptr = framebuffer;
    fb.enabled = 1;
    fb.dirty = 0;
}

typedef struct _file_browser_menu_args {
    char type;
    char path[512];
    char label[512];
} file_browser_menu_args_t;

void find_parent_dir(char* path, char* parent) {
    size_t last_separator = 0;
    for (size_t index = 0; index < strlen(path); index++) {
        if (path[index] == '/') last_separator = index;
    }

    strcpy(parent, path);
    parent[last_separator] = '\0';
}

bool ends_with(const char* input, const char* end) {
    size_t input_length = strlen(input);
    size_t end_length = strlen(end);
    for (size_t position = 0; position < end_length; position++) {
        if (input[input_length - position - 1] != end[end_length - position - 1]) {
            return false;
        }
    }
    return true;
}

bool file_browser(const char* initial_path, char* selected_file) {
    bool result = false;
    char path[512] = {0};
    strncpy(path, initial_path, sizeof(path));
    bool selected = false;
    while (!selected) {
        menu_t* menu = menu_alloc(path, 20, 18);
        DIR*    dir  = opendir(path);
        if (dir == NULL) {
            if (path[0] != 0) {
                ESP_LOGE(TAG, "Failed to open directory %s", path);
                display_fatal_error("Failed to open directory", NULL, NULL, NULL);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            return false;
        }
        struct dirent*            ent;
        file_browser_menu_args_t* pd_args = malloc(sizeof(file_browser_menu_args_t));
        pd_args->type                     = 'd';
        find_parent_dir(path, pd_args->path);
        menu_insert_item(menu, "../", NULL, pd_args, -1);

        while ((ent = readdir(dir)) != NULL) {
            file_browser_menu_args_t* args = malloc(sizeof(file_browser_menu_args_t));
            sprintf(args->path, path);
            if (path[strlen(path) - 1] != '/') {
                strcat(args->path, "/");
            }
            strcat(args->path, ent->d_name);

            if (ent->d_type == DT_REG) {
                args->type = 'f';
            } else {
                args->type = 'd';
            }

            snprintf(args->label, sizeof(args->label), "%s%s", ent->d_name, (args->type == 'd') ? "/" : "");
            
            if ((args->type == 'f') && (!(ends_with(ent->d_name, ".gb") || ends_with(ent->d_name, ".gbc")))) {
                free(args);
            } else {
                menu_insert_item(menu, args->label, NULL, args, -1);
            }
        }
        closedir(dir);

        bool                      render   = true;
        bool                      renderbg = true;
        bool                      exit     = false;
        file_browser_menu_args_t* menuArgs = NULL;

        while (!exit) {
            rp2040_input_message_t message = {0};
            if (xQueueReceive(button_queue, &message, 16 / portTICK_PERIOD_MS) == pdTRUE) {
                if (message.state) {
                    switch (message.input) {
                        case RP2040_INPUT_JOYSTICK_DOWN:
                            menu_navigate_next(menu);
                            render = true;
                            break;
                        case RP2040_INPUT_JOYSTICK_UP:
                            menu_navigate_previous(menu);
                            render = true;
                            break;
                        case RP2040_INPUT_BUTTON_BACK:
                            menuArgs = pd_args;
                            break;
                        case RP2040_INPUT_BUTTON_ACCEPT:
                        case RP2040_INPUT_JOYSTICK_PRESS:
                            menuArgs = menu_get_callback_args(menu, menu_get_position(menu));
                            break;
                        case RP2040_INPUT_BUTTON_HOME:
                            exit = true;
                            break;
                        default:
                            break;
                    }
                }
            }

            if (renderbg) {
                pax_background(&pax_buffer, 0xFFFFFF);
                pax_noclip(&pax_buffer);
                pax_draw_text(&pax_buffer, 0xFF000000, pax_font_saira_regular, 18, 5, 240 - 19, "🅰 select  🅱 back");
                renderbg = false;
            }

            if (render) {
                menu_render(&pax_buffer, menu, 0, 0, 320, 220);
                disp_flush();
                render = false;
            }

            if (menuArgs != NULL) {
                if (menuArgs->type == 'd') {
                    strcpy(path, menuArgs->path);
                    break;
                } else {
                    printf("File selected: %s\n", menuArgs->path);
                    strcpy(selected_file, menuArgs->path);
                    result = true;
                    exit = true;
                    selected = true;
                }
                menuArgs = NULL;
                render   = true;
                renderbg = true;
            }

            if (exit) {
                break;
            }
        }

        for (size_t index = 0; index < menu_get_length(menu); index++) {
            free(menu_get_callback_args(menu, index));
        }

        menu_free(menu);
    }
    return result;
}

bool load_rom(bool browser, bool sd_card) {
    if (browser) {
        if (!file_browser(sd_card ? "/sd" : "/internal", rom_filename)) return false;
    }
    
    display_state("Loading ROM...", 0);
    
    if (rom_data != NULL) {
        free(rom_data);
        rom_data = NULL;
    }

    FILE* rom_fd = fopen(rom_filename, "rb");
    if (rom_fd == NULL) {
        memset(rom_filename, 0, sizeof(rom_filename));
        nvs_set_str(nvs_handle_gnuboy, "rom", rom_filename);
        show_error("Failed to open ROM file", 100);
        return false;
    }

    size_t rom_length;
    rom_data = load_file_to_ram(rom_fd, &rom_length);
    fclose(rom_fd);

    if (rom_data == NULL) {
        memset(rom_filename, 0, sizeof(rom_filename));
        nvs_set_str(nvs_handle_gnuboy, "rom", rom_filename);
        show_error("Failed to load ROM file", 100);
        return false;
    }

    loader_init(rom_data);
    reset_and_init();
    lcd_begin();
    sound_reset();
    
    display_state("ROM loaded", 100);
    
    nvs_set_str(nvs_handle_gnuboy, "rom", rom_filename);
    printf("ROM filename stored: '%s'\n", rom_filename);

    load_sram();
    return true;
}

void game_loop() {
    if (rom_filename[0] == '\0') {
        show_error("No ROM loaded", 100);
        return;
    }
    pax_draw_image(&pax_buffer, &border, 0, 0);
    disp_flush();

    uint startTime;
    uint stopTime;
    uint totalElapsedTime = 0;
    uint actualFrameCount = 0;
        
    bool quit = false;
    while (!quit) {
        //startTime = xthal_get_ccount();
        run_to_vblank();
        /*stopTime = xthal_get_ccount();

        if (stopTime > startTime) {
            elapsedTime = (stopTime - startTime);
        } else {
            elapsedTime = ((uint64_t)stopTime + (uint64_t)0xffffffff) - (startTime);
        }

        totalElapsedTime += elapsedTime;*/
        ++frame;
        /*++actualFrameCount;

        if (actualFrameCount == 60) {
          float seconds = totalElapsedTime / (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000.0f); // 240000000.0f; // (240Mhz)
          float fps = actualFrameCount / seconds;

          printf("HEAP:0x%x, FPS:%f\n", esp_get_free_heap_size(), fps);

          actualFrameCount = 0;
          totalElapsedTime = 0;
        }*/
        
        rp2040_input_message_t buttonMessage = {0};
        BaseType_t queueResult;
        do {
            queueResult = xQueueReceive(button_queue, &buttonMessage, 0);
            if (queueResult == pdTRUE) {
                uint8_t button = buttonMessage.input;
                bool value = buttonMessage.state;
                switch(button) {
                    case RP2040_INPUT_JOYSTICK_DOWN:
                        pad_set(PAD_DOWN, value);
                        break;
                    case RP2040_INPUT_JOYSTICK_UP:
                        pad_set(PAD_UP, value);
                        break;
                    case RP2040_INPUT_JOYSTICK_LEFT:
                        pad_set(PAD_LEFT, value);
                        break;
                    case RP2040_INPUT_JOYSTICK_RIGHT:
                        pad_set(PAD_RIGHT, value);
                        break;
                    case RP2040_INPUT_BUTTON_ACCEPT:
                        pad_set(PAD_A, value);
                        break;
                    case RP2040_INPUT_BUTTON_BACK:
                        pad_set(PAD_B, value);
                        break;
                    case RP2040_INPUT_BUTTON_START:
                        pad_set(PAD_START, value);
                        break;
                    case RP2040_INPUT_BUTTON_SELECT:
                        pad_set(PAD_SELECT, value);
                        break;
                    case RP2040_INPUT_BUTTON_HOME:
                        if (value) {
                            audio_stop();
                            save_sram();
                            save_state();
                            exit_to_launcher();
                        }
                        break;
                    case RP2040_INPUT_BUTTON_MENU:
                        if (value) {
                            quit = true;
                        }
                    default:
                        break;
                }
            }
        } while (queueResult == pdTRUE);
    }
}

void app_main(void) {
    bsp_init();
    ili9341 = get_ili9341();
    pax_buf_init(&pax_buffer, NULL, 320, 240, PAX_BUF_16_565RGB);

    pax_decode_png_buf(&border, (void*) border_png_start, border_png_end - border_png_start, PAX_BUF_16_565RGB, 0);

    display_state("Initializing...", 0);
    
    bsp_rp2040_init();
    button_queue = get_rp2040()->queue;

    nvs_flash_init();
    
    esp_err_t res;
    
    displayBuffer[0] = heap_caps_malloc(160 * 144 * 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    displayBuffer[1] = heap_caps_malloc(160 * 144 * 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    framebuffer = displayBuffer[0];
    currentBuffer = 0;
    
    if (displayBuffer[0] == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fb0!");
        display_fatal_error("Failed to allocate fb0!", NULL, NULL, NULL);
        exit_to_launcher();
    }

    if (displayBuffer[1] == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fb1!");
        display_fatal_error("Failed to allocate fb1!", NULL, NULL, NULL);
        exit_to_launcher();
    }
    
    for (int i = 0; i < 2; ++i){
        memset(displayBuffer[i], 0, 160 * 144 * 2);
    }
    
    printf("app_main: displayBuffer[0]=%p, [1]=%p\n", displayBuffer[0], displayBuffer[1]);
    
    const size_t lineSize = ILI9341_WIDTH * LINE_COUNT * sizeof(uint16_t);
    for (int x = 0; x < LINE_BUFFERS; x++)
    {
        line[x] = heap_caps_malloc(lineSize, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (line[x] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate line %u!", x);
            display_fatal_error("Failed to allocate line!", NULL, NULL, NULL);
            exit_to_launcher();
        }
        memset(line[x], 0, lineSize);
    }
    
    /* Start internal filesystem */
    if (mount_internal_filesystem() != ESP_OK) {
        display_fatal_error("An error occured", "Failed to initialize flash FS", NULL, NULL);
        exit_to_launcher();
    }

    /* Start SD card filesystem */
    bool sdcard_mounted = (mount_sdcard_filesystem() == ESP_OK);
    if (sdcard_mounted) {
        ESP_LOGI(TAG, "SD card filesystem mounted");
        /* LED power is on: start LED driver and turn LEDs off */
        ws2812_init(GPIO_LED_DATA);
        const uint8_t led_off[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        ws2812_send_data(led_off, sizeof(led_off));
    } else {
        gpio_set_level(GPIO_SD_PWR, 0);  // Disable power to LEDs and SD card
    }
    
    audio_init(AUDIO_SAMPLE_RATE);

    vidQueue = xQueueCreate(1, sizeof(uint16_t*));
    xTaskCreatePinnedToCore(&videoTask, "videoTask", 4096, NULL, 5, NULL, 1);
    audioQueue = xQueueCreate(1, sizeof(uint16_t*));
    xTaskCreatePinnedToCore(&audioTask, "audioTask", 2048, NULL, 5, NULL, 1); //768

    reset_and_init();

    // pcm.len = count of 16bit samples (x2 for stereo)
    memset(&pcm, 0, sizeof(pcm));
    pcm.hz = AUDIO_SAMPLE_RATE;
    pcm.stereo = 1;
    pcm.len = /*pcm.hz / 2*/ audioBufferLength;
    pcm.buf = (int16_t*) heap_caps_malloc(AUDIO_BUFFER_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    pcm.pos = 0;

    audioBuffer[0] = (int32_t*) pcm.buf;
    audioBuffer[1] = (int32_t*) heap_caps_malloc(AUDIO_BUFFER_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    
    if (audioBuffer[0] == NULL) {
        show_error("Failed to allocate audio buffer 0", 100);
        exit_to_launcher();
    }
    
    if (audioBuffer[1] == NULL) {
        show_error("Failed to allocate audio buffer 1", 100);
        exit_to_launcher();
    }

    res = nvs_open("gnuboy", NVS_READWRITE, &nvs_handle_gnuboy);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %d", res);
    }
    
    nvs_get_str_fixed(nvs_handle_gnuboy, "rom", rom_filename, sizeof(rom_filename) - 1, NULL);

    printf("ROM filename: '%s'\n", rom_filename);

    if (rom_filename[0] != '\0') {
        load_rom(false, false);
        load_sram();
        load_state();
        game_loop();
    }
    
    int volume;
    if (nvs_get_i32(nvs_handle_gnuboy, "volume", &volume) == ESP_OK) {
        audio_volume_set(volume);
    }

    while(true) {
        audio_stop();
        menu_action_t action = show_menu();
        switch(action) {
            case ACTION_EXIT:
                if (rom_filename[0] != '\0') {
                    audio_stop();
                    save_sram();
                    save_state();
                }
                exit_to_launcher();
                break;
            case ACTION_LOAD_ROM:
                if (rom_filename[0] != '\0') {
                    audio_stop();
                    save_sram();
                    save_state();
                }
                if (load_rom(true, true)) {
                    load_state();
                    audio_resume();
                    game_loop();
                }
                break;
            case ACTION_LOAD_ROM_INT:
                if (rom_filename[0] != '\0') {
                    audio_stop();
                    save_sram();
                    save_state();
                }
                if (load_rom(true, false)) {
                    load_state();
                    audio_resume();
                    game_loop();
                }
                break;
            case ACTION_LOAD_SAVE:
                load_sram();
                break;
            case ACTION_STORE_SAVE:
                save_sram();
                break;
            case ACTION_LOAD_STATE:
                load_state();
                break;
            case ACTION_STORE_STATE:
                save_state();
                break;
            case ACTION_RUN:
                audio_resume();
                game_loop();
                break;
            case ACTION_RESET:
                if (rom_filename[0] == '\0') {
                    show_error("No ROM loaded", 100);
                } else {
                    load_rom(false, false);
                    save_state();
                    audio_resume();
                    game_loop();
                }
                break;
            case ACTION_VOLUME_DOWN: {
                int volume = audio_volume_decrease();
                nvs_set_i32(nvs_handle_gnuboy, "volume", volume);
                char message[32];
                message[31] = '\0';
                uint8_t volume_percent = 0;
                if (volume == 1) volume_percent = 25;
                if (volume == 2) volume_percent = 50;
                if (volume == 3) volume_percent = 75;
                if (volume == 4) volume_percent = 100;
                snprintf(message, sizeof(message) - 1, "Volume set to %u%%\n", volume_percent);
                show_message(message, 50);
                break;
            }
            case ACTION_VOLUME_UP: {
                int volume = audio_volume_increase();
                nvs_set_i32(nvs_handle_gnuboy, "volume", volume);
                char message[32];
                message[31] = '\0';
                uint8_t volume_percent = 0;
                if (volume == 1) volume_percent = 25;
                if (volume == 2) volume_percent = 50;
                if (volume == 3) volume_percent = 75;
                if (volume == 4) volume_percent = 100;
                snprintf(message, sizeof(message) - 1, "Volume set to %u%%\n", volume_percent);
                show_message(message, 50);
                break;
            }
            default:
                ESP_LOGW(TAG, "Action %u", (uint8_t) action);
        }
    }
    
    load_sram();
}
