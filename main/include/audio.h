#ifndef AUDIO_H
#define AUDIO_H

#define I2S_NUM     I2S_NUM_0
#define VOLUME_LEVEL_COUNT (5)

typedef enum
{
    VOLUME_LEVEL0 = 0,
    VOLUME_LEVEL1 = 1,
    VOLUME_LEVEL2 = 2,
    VOLUME_LEVEL3 = 3,
    VOLUME_LEVEL4 = 4,

    _VOLUME_FILLER = 0xffffffff
} volume_level;


void audio_init(int sample_rate);
void audio_submit(short* stereoAudioBuffer, int frameCount);
void audio_stop();
void audio_resume();
volume_level audio_volume_get();
void audio_volume_set(volume_level value);
int audio_volume_change();
int audio_volume_increase();
int audio_volume_decrease();

#endif
