#include "resource.h"

//IMAGE
ResourceImage::ResourceImage(RESOURCE_ID id) : Resource(id)
{
	_textureID = NULL;
	const ResourceDescriptor * desc = &RESOURCES_DESCRIPTOR[(int)_id];
	
	// generating textures
	_textureID = new GLuint[_count];
	memset(_textureID, 0, sizeof(GLuint)*_count);

}

int ResourceImage::loadThreaded(void * data){
	Resource::sThreadData * t_data = (Resource::sThreadData*)data;
	ResourceImage * _this = (ResourceImage*)t_data->r;
	 
	unsigned char * image = NULL;
	unsigned int w, h;
	unsigned char * png = NULL;
	size_t png_size;
	unsigned int error;
	LodePNGState state;
	lodepng_state_init(&state);
	//loading png from file
	t_data->data = NULL;
	error = lodepng_load_file(&png, &png_size, t_data->path);
	if(error == 0){
		error = lodepng_decode(&image, &w, &h, &state, png, png_size);
		t_data->image.w = w;
		t_data->image.h = h;
		t_data->data = image;
	}

	// remove data
	free(png);

	// getting descriptor
	const ResourceDescriptor * desc = &RESOURCES_DESCRIPTOR[(int)_this->_id];

	if(!error){
		//getting channel information
		int channels = lodepng_get_channels(&state.info_png.color);
		//creating texture
		if(channels <= 0 || channels > 4){
			ERROR_F("Unable to create texture from png: Invalid number of channels (%d)!", channels);
			error = 1;
		}
		else
		{
			GLint format = GL_RGBA;
			switch(channels)
			{
			case 1:
			format = GL_RED;
			break;
			case 2:
			format = GL_RG;
			break;
			case 3:
			format = GL_RGB;
			case 4:
			default:{
				if(desc->options.image.pre_mult_alpha){
					optimizeBorderAlpha((Uint32*)image, w, h);
				}
			}
			}
			t_data->image.format = format;
		}
	}
	// cleaning up state
	lodepng_state_cleanup(&state);

	// signaling finished
	RESOURCE_THREAD_FINISH_SIGNALING(t_data->thread_id);

	return error;
}

int ResourceImage::load(const sThreadData * data){
	if(data->data != NULL){
		glGenTextures(1, &_textureID[data->offset]);// performance improvement: generating texture right before it is beeing filled by glTexImage2D()
		loadFromData(data->image.w, data->image.h, data->data, GL_UNSIGNED_BYTE, data->image.format, data->offset);
		setTextureParameters(data->offset);
		if(data->offset == _count-1)// last texture successfully loaded
			_loaded = true;
		free(data->data);
		return 0;
	}
	return 1;
}

void ResourceImage::setTextureParameters(int id_offset){
	const ResourceDescriptor * desc = &RESOURCES_DESCRIPTOR[(int)_id];
	GLint wrap;
	GLint filter;
	switch(desc->options.image.wrap){
		case IMAGE_WRAP_REPEAT:{
			wrap = GL_REPEAT;
		}break;
		case IMAGE_WRAP_CLAMP:
		default:{
			wrap = GL_CLAMP_TO_EDGE;
		}
	}
	glBindTexture(GL_TEXTURE_2D, _textureID[id_offset]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); 
	GLint min_filter = GL_LINEAR;
	if(desc->options.image.gen_mipmap) {// mipmap generation demanded?
		if(GLEW_SGIS_generate_mipmap){// check whether mipmap generation is supported
			glGenerateMipmap(GL_TEXTURE_2D);
			min_filter = GL_LINEAR_MIPMAP_LINEAR;
		}
	}
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter); 
	glBindTexture(GL_TEXTURE_2D, 0);
}

void ResourceImage::loadFromData(int w, int h, unsigned char * pixels, GLenum type, GLenum format, int id_offset){
	// change unpack-alignment if necessary
	switch(format)
	{
	case GL_RED:
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	break;
	case GL_RG:
	glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
	break;
	case GL_RGB:
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	break;
	}
	glBindTexture(GL_TEXTURE_2D, _textureID[id_offset]);
	glTexImage2D(GL_TEXTURE_2D, 0, format, w, h, 0, format, type, pixels);
	glBindTexture(GL_TEXTURE_2D, 0);

	// reset unpack alignment
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

	_width = w;
	_height = h;
}

ResourceImage::~ResourceImage()
{
	if(_textureID != NULL){
		glDeleteTextures(_count, _textureID);
		delete[] _textureID;
	}
	
}

void ResourceImage::optimizeBorderAlpha(Uint32 * pixels, int width, int height)
{
	int num_pixels = width*height;
	for(int i = 0; i < num_pixels; i++)
	{
		Uint8 * rgba_center = (Uint8*)(pixels + i);
		if(rgba_center[3] == 0)
		{
			Uint8 * rgba_neighbour = NULL;
			int x = i%width;
			int y = i/width;
			int count = 0;
			int r = 0,g = 0,b = 0;
			if(x != 0)
			{
				rgba_neighbour = (Uint8*)(pixels + (i-1));
				if(rgba_neighbour[3] > 0)//not completely transparent
				{
					r += rgba_neighbour[0];
					g += rgba_neighbour[1];
					b += rgba_neighbour[2];
					count++;
				}
			}
			if(x != width-1)
			{
				rgba_neighbour = (Uint8*)(pixels + (i+1));
				if(rgba_neighbour[3] > 0)//not completely transparent
				{
					r += rgba_neighbour[0];
					g += rgba_neighbour[1];
					b += rgba_neighbour[2];
					count++;
				}
			}
			if(y != 0)
			{
				rgba_neighbour = (Uint8*)(pixels + (i-width));
				if(rgba_neighbour[3] > 0)//not completely transparent
				{
					r += rgba_neighbour[0];
					g += rgba_neighbour[1];
					b += rgba_neighbour[2];
					count++;
				}
			}
			if(y != height-1)
			{
				rgba_neighbour = (Uint8*)(pixels + (i+width));
				if(rgba_neighbour[3] > 0)//not completely transparent
				{
					r += rgba_neighbour[0];
					g += rgba_neighbour[1];
					b += rgba_neighbour[2];
					count++;
				}
			}
			if(count != 0)
			{
				r /= count;
				g /= count;
				b /= count;
			}
			rgba_center[0] = r;
			rgba_center[1] = g;
			rgba_center[2] = b;
		}
	}
}
