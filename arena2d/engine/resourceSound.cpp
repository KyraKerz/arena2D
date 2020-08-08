#include "resource.h"

// SOUND
ResourceSound::ResourceSound(RESOURCE_ID id) : Resource(id)
{
	_buffer = new ALuint[_count];
	memset(_buffer, 0, sizeof(ALuint)*_count);
}

ResourceSound::~ResourceSound()
{
	if(_buffer != NULL){
		alDeleteBuffers(_count, _buffer);
		_buffer = NULL;
		delete[] _buffer;
	}
}

int ResourceSound::loadThreaded(void * thread_data)
{
	Resource::sThreadData * t = (Resource::sThreadData*)thread_data;
	t->data = NULL;
	ResourceSound * _this = (ResourceSound*)t->r;
	// loading WAV
	ALuint chan = 0;
	ALuint samplerate = 0;
	ALuint bps = 0;
	ALuint dataSize = 0;
	uint16_t BitsPerSample = 0;
	//open file for reading
	FILE *f = fopen(t->path, "rb");
	if (f == NULL) {
		return 1;
	}
	char buffer[4];
	fread(&buffer, 1, 4, f);
	if (memcmp(buffer, "RIFF", 4) != 0) {
		fclose(f);
		ERROR_F("Error while loading %s: Invalid WAV file!", t->path);
		return 1;
	}
	fread(&buffer, 1, 4, f); //File size
	fread(&buffer, 1, 4, f); //"Wave"
	fread(&buffer, 1, 4, f); //"fmt "
	fread(&buffer, 1, 4, f); //"16"
	fread(&buffer, 1, 2, f); //Format tag: 1=PCM
	fread(&buffer, 1, 2, f); //Cannels(1=Mono/2=Sterio)
	chan = *(ALuint*)buffer;
	fread(&buffer, 1, 4, f); //Samplerate
	samplerate = *(ALuint*)buffer;
	fread(&buffer, 1, 4, f); //Bytes/Second1
	bps = *(ALuint*)buffer;
	fread(&buffer, 1, 2, f); //Frame Größe
	fread(&buffer, 1, 2, f); //Bits/Sample
	BitsPerSample = *(ALushort *)buffer;

	fread(&buffer, 4, 1, f);
	if (memcmp(buffer, "LIST", 4) == 0)//LIST Signatur
	{
		fread(&buffer, 4, 1, f); //LIST Chunk size
		ALuint chunkSize = *(ALuint*)buffer;
		fseek(f, chunkSize, SEEK_CUR); //Skip whole LIST Chunk
		fread(&buffer, 1, 4, f); //Data Header
	}
	else if (memcmp(buffer, "data", 4) != 0)//no data found -> unknown format
	{
		fclose(f);
		ERROR_F("Error whle loading %s: Invalid Signature '%c%c%c%c'", buffer[0], buffer[1], buffer[2], buffer[3]);
		return 1;
	}
	// getting data
	fread(&buffer, 1, 4, f); //Data length
	dataSize = *(ALuint*)buffer;
	unsigned char * data = new unsigned char[dataSize];
	fread(data, 1, dataSize, f);
	fclose(f);
	t->data = data;

	// setting format
	if (chan == 1)
		t->sound.format = AL_FORMAT_MONO16;
	else
		t->sound.format = AL_FORMAT_STEREO16;

	// setting size and samplerate
	t->sound.size = dataSize;
	t->sound.samplerate = samplerate;

	// sending signal that thread is finished
	RESOURCE_THREAD_FINISH_SIGNALING(t->thread_id);

	return 0;
}


int ResourceSound::load(const sThreadData * data)
{
	if(data->data == NULL)
		return 1;
	// create buffer
	alGenBuffers(1, &_buffer[data->offset]);
	alBufferData(_buffer[data->offset], data->sound.format, (const ALvoid*)data->data, data->sound.size, data->sound.samplerate);
	delete[](data->data);
	return 0;
}
