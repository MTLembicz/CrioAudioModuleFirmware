#include "main.h"
#include "stm32f1xx_hal.h"

#define AUDIO_BUFFER_SIZE		1000
#define WAV_FILE_HEADER_SIZE	44

#define CHUNK_ID_CONST			0x46464952
#define FORMAT_CONST			0x45564157
#define CHANNEL_STEREO			2

typedef enum {
  WAV_STATE_IDLE = 0,
  WAV_STATE_INIT,
  WAV_STATE_START,
  WAV_STATE_PLAY,
  WAV_STATE_VOLUME_UP,
  WAV_STATE_VOLUME_DOWN,
  WAV_STATE_ERROR,
}WavPlayerState;

typedef enum {
  WAV_FILE_START = 0,
  WAV_FILE_FINISH,
  WAV_FILE_ALARM,
}WavFileSelect;

struct WavFilesInfo
{
	uint32_t chunkId;
	uint32_t chunkSize;
	uint32_t format;
	uint32_t subchunk1Id;
	uint32_t subchunk1Size;
	uint16_t audioFormat;
	uint16_t numChannels;
	uint32_t sampleRate;
	uint32_t byteRate;
	uint16_t blockAlign;
	uint16_t bitsPerSample;
	uint32_t subchunk2Id;
	uint32_t subchunk2Size;
};

void DACConfigureI2SFormat(SPI_HandleTypeDef *hspi);
void DACSetVolume(uint16_t volume);
bool SDMount(void);
bool WAVPlayerFileSelect(const char* filePath);
void WAVPlayerPlay(I2S_HandleTypeDef* i2s);
void WAVPlayerBufferState(uint8_t bs);
void WAVPlayerProcess(I2S_HandleTypeDef* i2s);
bool WAVPlayerStopAndCloseFile(void);
void WAVPlayerFillBufferHalf(void);
void WAVPlayerFillBufferFull(void);
