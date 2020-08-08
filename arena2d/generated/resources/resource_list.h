// this file is generated automatically, DO NOT EDIT!
#ifndef RESOURCE_LIST_H
#define RESOURCE_LIST_H

#define RESOURCE_TYPE_IMAGE 0
#define RESOURCE_TYPE_SOUND 1
#define RESOURCE_TYPE_FONT 2

#define IMAGE_WRAP_REPEAT 0
#define IMAGE_WRAP_CLAMP 1

union ResourceOptions{
	struct Image{
		Image(bool _pre_mult_alpha, bool _gen_mipmap, bool _wrap): pre_mult_alpha(_pre_mult_alpha), gen_mipmap(_gen_mipmap), wrap(_wrap){}
		bool pre_mult_alpha;
		bool gen_mipmap;
		int wrap;
	}image;
	struct Sound{
		Sound(bool _compress): compress(_compress){}
		bool compress;
	}sound;
	struct Font{
	Font(int _outline): outline(_outline){}
		int outline;
	}font;
	ResourceOptions(const Image & i): image(i){}
	ResourceOptions(const Sound & s): sound(s){}
	ResourceOptions(const Font & f): font(f){}};

struct ResourceDescriptor{
	ResourceDescriptor(int _type, const char * _path, int _start_count, int _end_count, const ResourceOptions & _options): type(_type), path(_path), start_count(_start_count), end_count(_end_count), options(_options){}
	int type;
	const char * path;
	int start_count;// start number for file collection (e.g. image000.png -> 0), ignored if negative
	int end_count;// end number for file collection (e.g. image011.png -> 11), ignored if negative

	// resource options
	ResourceOptions options;
};

enum class RESOURCE_ID{
	FONT_MONOSPACE_REGULAR=0,
	FONT_MONOSPACE_BOLD=1
};

#define RESOURCES_NUMBER 2
extern const ResourceDescriptor RESOURCES_DESCRIPTOR[RESOURCES_NUMBER];
#endif