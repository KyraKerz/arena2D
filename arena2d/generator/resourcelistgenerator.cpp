#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <errno.h>
#include <vector>

#define LINE_BUFFER_SIZE 1024

#define RESOURCE_TYPE_IMAGE 0
#define RESOURCE_TYPE_SOUND 1
#define RESOURCE_TYPE_FONT 2
#define RESOURCE_TYPE_NUM 3
const char *RESOURCE_TYPE_NAMES[RESOURCE_TYPE_NUM] ={// names as written in the ResourceOptions union
	"Image",
	"Sound",
	"Font"
};

#define RESOURCE_IMAGE_WRAP_REPEAT 0
#define RESOURCE_IMAGE_WRAP_CLAMP 1

const char * BOOL_STRING[2] = {"false", "true"};
const char * HEADER_PRE = 
	"#ifndef RESOURCE_LIST_H\n"
	"#define RESOURCE_LIST_H\n"
	"\n"
	"#define RESOURCE_TYPE_IMAGE 0\n"
	"#define RESOURCE_TYPE_SOUND 1\n"
	"#define RESOURCE_TYPE_FONT 2\n"
	"\n"
	"#define IMAGE_WRAP_REPEAT 0\n"
	"#define IMAGE_WRAP_CLAMP 1\n"
	"\n"
	"union ResourceOptions{\n"
	"	struct Image{\n"
	"		Image(bool _pre_mult_alpha, bool _gen_mipmap, bool _wrap): pre_mult_alpha(_pre_mult_alpha), gen_mipmap(_gen_mipmap), wrap(_wrap){}\n"
	"		bool pre_mult_alpha;\n"
	"		bool gen_mipmap;\n"
	"		int wrap;\n"
	"	}image;\n"
	"	struct Sound{\n"
	"		Sound(bool _compress): compress(_compress){}\n"
	"		bool compress;\n"
	"	}sound;\n"
	"	struct Font{\n"
	"	Font(int _outline): outline(_outline){}\n"
	"		int outline;\n"
	"	}font;\n"
	"	ResourceOptions(const Image & i): image(i){}\n"
	"	ResourceOptions(const Sound & s): sound(s){}\n"
	"	ResourceOptions(const Font & f): font(f){}"
	"};\n"
	"\n"
	"struct ResourceDescriptor{\n"
	"	ResourceDescriptor(int _type, const char * _path, int _start_count, int _end_count, const ResourceOptions & _options): type(_type), path(_path), start_count(_start_count), end_count(_end_count), options(_options){}\n"
	"	int type;\n"
	"	const char * path;\n"
	"	int start_count;// start number for file collection (e.g. image000.png -> 0), ignored if negative\n"
	"	int end_count;// end number for file collection (e.g. image011.png -> 11), ignored if negative\n"
	"\n"
	"	// resource options\n"
	"	ResourceOptions options;\n"
	"};\n"
	"\n"
	"enum class RESOURCE_ID{\n";

const char * HEADER_POST = 
	"\n};\n";

const char * AUTO_DISCLAIMER = 
	"// this file is generated automatically, DO NOT EDIT!\n";

const char * CPP_PRE = 
	"const ResourceDescriptor RESOURCES_DESCRIPTOR[RESOURCES_NUMBER] = {\n";

const char * CPP_POST = 
	"\n};";

int isAlphaNumeric(char c){
	return 	(c >= 'a' && c <= 'z') ||
			(c >= 'A' && c <= 'Z') ||
			(c >= '0' && c <= '9') ||
			c == '_';
}

// return: pointer to end of token in text, or NULL if no token extracted
char* getNextToken(char * text, char * token)
{
	if(text == NULL){
		token[0] = '\0';
		return NULL;
	}
	// skipping white spaces
	while(	*text == ' '|| 
			*text == '\t')text++;

	if(*text == '\0' || *text == '\n' || *text == '\r'){ // nothing here
		token[0] = '\0';
		return NULL;
	}
	
	// check if token starts with "
	if(*text == '"')
	{
		do{
			*token = *text;
			token++;
			text++;
		}
		while(	*text != '\0' &&
				*text != '\n' &&
				*text != '\r' &&
				*text != '"');
		if(*text == '"'){
			*token = '"';
			token++;
			text++;
		}
		*token = '\0';
	}
	else if(isAlphaNumeric(*text))
	{
		do{
			*token = *text;
			token++;
			text++;
		}
		while(isAlphaNumeric(*text));
		*token = '\0';
	}
	else// any other special character
	{
		token[0] = text[0];
		token[1] = '\0';
		text++;
	}

	return text;

}

// returns 0 if string is valid
int checkStringToken(char * token, int token_len, const char * previous_token, int line){
	if(token[0] != '"'){
		fprintf(stderr, "Line %d: Expected string (\"\") after '%s'", line, previous_token);
		return 1;
	}
	else if(token_len < 2 || token[token_len-1] != '"') {
		fprintf(stderr, "Line %d: Missing closing '\"' in string token\n", line);
		return 1;
	}

	return 0;
}

struct ResourceAttribute{
	enum Type{INT, BOOL, IMAGE_WRAP};
	ResourceAttribute(const char * _name, Type t, int _default_value) :
					name(_name), type(t), default_value(_default_value), value(_default_value){}
	void reset(){value = default_value;}
	void makeDefault(){default_value = value;}
	int default_value;
	int value;
	const char * name;
	Type type;
	void print(FILE * f){
		switch(type){
			case IMAGE_WRAP:
			case INT:{
					fprintf(f, "%d", value);
			}break;
			case BOOL:{
				if(value)
					fprintf(f, "true");
				else
					fprintf(f, "false");
			}break;
			default:{
				fputs("<Not implemented!>", f);
			}
		}
	}
	// parsing attribute value, returns 0 on success, 1 on error
	int parse(const char * str){
		switch(type){
			case IMAGE_WRAP:{
				if(!strcmp(str, "CLAMP")){
					value = RESOURCE_IMAGE_WRAP_CLAMP;
				}
				else if(!strcmp(str, "REPEAT")){
					value = RESOURCE_IMAGE_WRAP_REPEAT;
				}
				else
					return 1;
				return 0;
			}break;
			case INT:{
				value = atoi(str);
			}break;
			case BOOL:{
				if(!strcmp(str, "true") ||
					!strcmp(str, "TRUE")){
					value = 1;
				}
				else if(!strcmp(str, "false") ||
						!strcmp(str, "FALSE")){
					value = 0;
				}else{
					return 1;
				}
				return 0;
			}break;
			default:{
				return 1;
			}
		}
		return 1;
	}
};

int main(int argc, char ** args)
{
	if(argc != 4)
	{
		printf("usage: %s <input> <header_file> <cpp_file>\n", args[0]);
		return 1;
	}

	// open file for reading
	FILE * input = fopen(args[1], "r");
	if(input == NULL) {
		fprintf(stderr, "Could not open resource file '%s': %s\n", args[1], strerror(errno));
		return 1;
	}

	// open header output file
	FILE * output_header = fopen(args[2], "w");
	if(output_header == NULL) {
		fprintf(stderr, "Could not open header file '%s' for writing: %s\n", args[2], strerror(errno));
		return 1;
	}

	// open cpp output file
	FILE * output_cpp = fopen(args[3], "w");
	if(output_cpp == NULL) {
		fprintf(stderr, "Could not open cpp file '%s' for writing: %s\n", args[3], strerror(errno));
		return 1;
	}

	// write header prefix
	fputs(AUTO_DISCLAIMER, output_header);
	fputs(HEADER_PRE, output_header);

	// write cpp prefix
	fputs(AUTO_DISCLAIMER, output_cpp);
	fprintf(output_cpp, "#include \"%s\"\n\n", args[2]);
	fputs(CPP_PRE, output_cpp);

	//reading line by line
	char line[LINE_BUFFER_SIZE];
	char token[LINE_BUFFER_SIZE];
	char path_prefix[LINE_BUFFER_SIZE] = "";
	char resource_id[LINE_BUFFER_SIZE];
	int line_count = 0;
	int error = 0;
	int num_resources = 0;

	// resource attributes
	std::vector<ResourceAttribute> attribs[RESOURCE_TYPE_NUM];
	// image
	attribs[RESOURCE_TYPE_IMAGE].push_back(ResourceAttribute("pre_mult_alpha", ResourceAttribute::BOOL, 0));
	attribs[RESOURCE_TYPE_IMAGE].push_back(ResourceAttribute("gen_mipmap", ResourceAttribute::BOOL, 0));
	attribs[RESOURCE_TYPE_IMAGE].push_back(ResourceAttribute("wrap", ResourceAttribute::IMAGE_WRAP, RESOURCE_IMAGE_WRAP_CLAMP));
	// sound
	attribs[RESOURCE_TYPE_SOUND].push_back(ResourceAttribute("compress", ResourceAttribute::BOOL, 0));
	// font
	attribs[RESOURCE_TYPE_FONT].push_back(ResourceAttribute("outline", ResourceAttribute::INT, 0));

	while(fgets(line, LINE_BUFFER_SIZE, input)){
		line_count++;
		int resource_type = -1; 
		const char * resource_name = "";
		// get first token
		char * line_index = getNextToken(line, token);
		if(line_index == NULL)// no token in line, ignore
			continue;
		else if(!strcmp(token, "#"))// comment, ignore rest of line
			continue;
		else if(!strcmp(token, "img")){
			resource_type = RESOURCE_TYPE_IMAGE;
			resource_name = "img";
		}
		else if(!strcmp(token, "snd")){
			resource_type = RESOURCE_TYPE_SOUND;
			resource_name = "snd";
		}
		else if(!strcmp(token, "fnt")){
			resource_type = RESOURCE_TYPE_FONT;
			resource_name = "fnt";
		}
		else if(!strcmp(token, "prefix")){// setting new prefix
			line_index = getNextToken(line_index, token);
			if(!strcmp(token, ":")){
				line_index = getNextToken(line_index, token);
				int token_len = strlen(token);
				if(!checkStringToken(token, token_len, ":", line_count)){
					token[token_len-1] = '\0';
					strcpy(path_prefix, token+1);
					if(getNextToken(line_index, token)) {
						fprintf(stderr, "Line %d: Expected EOL after string\n", line_count); 
						error = 1;
					}
				}
				else
					error = 1;
			}
			else{
				error = 1;
				fprintf(stderr, "Line %d: Expected ':' after 'prefix'\n", line_count);
			}
		}
		else{
			fprintf(stderr, "Line %d: Unknown opening statement '%s'\n", line_count, token);
			error = 1;
		}

		if(resource_type >= 0)// a resource is declared in current line
		{
			// expecting resource name
			line_index = getNextToken(line_index, token);
			if(line_index != NULL && isAlphaNumeric(token[0]))
			{
				// checking if this is a default declaration
				int is_default = 0;
				if(!strcmp(token, "DEFAULT")) {
					is_default = 1;
				}
				else{
					strcpy(resource_id, token);
				}
				
				if(!is_default)
				{
				}
				int start_count = -1, end_count = -1;
				// optional collection []
				line_index = getNextToken(line_index, token);
				if(!strcmp(token, "[")){
					line_index = getNextToken(line_index, token);
					if(isAlphaNumeric(token[0])){
						start_count = atoi(token);
						line_index = getNextToken(line_index, token);
						if(!strcmp(token, "-")){
							line_index = getNextToken(line_index, token);
							if(isAlphaNumeric(token[0])){
								end_count = atoi(token);
								line_index = getNextToken(line_index, token);
								if(!strcmp(token, "]")){
									line_index = getNextToken(line_index, token);
								}
								else{
									error = 1;
									fprintf(stderr, "Line %d: Expected closing bracket ']' after numeric value\n", line_count);
								}
							}
							else{
								error = 1;
								fprintf(stderr, "Line %d: Expected numeric value after '-'\n", line_count);	
							}
						}
						else{
							error = 1;
							fprintf(stderr, "Line %d: Expected '-' after numeric value\n", line_count);	
						}
					}
					else{
						error = 1;
						fprintf(stderr, "Line %d: Expected numeric value after '['\n", line_count);	
					}
				}

				// optional options ()
				if(!error && !strcmp(token, "(")){
					line_index = getNextToken(line_index, token);
					do{
						// finding attribute
						std::vector<ResourceAttribute>::iterator it = attribs[resource_type].begin();
						while(it != attribs[resource_type].end())
						{
							if(!strcmp(token, it->name))
								break;
							it++;
						}
						if(it == attribs[resource_type].end()) { // no such attribute found!
							error = 1;
							fprintf(stderr, "Line %d: Unknown resource attribute '%s'\n", line_count, token);
							break;
						}

						// expecting =
						line_index = getNextToken(line_index, token);
						if(!strcmp(token, "=")){
						}else{
							error = 1;
							fprintf(stderr, "Line %d: Expected '=' after attribute name\n", line_count);
							break;
						}

						//expecting value
						line_index = getNextToken(line_index, token);
						if(it->parse(token)){
							error = 1;
							fprintf(stderr, "Line %d: Unable to parse value of attribute '%s'\n", line_count, it->name);
							break;
						}
						if(is_default){
							it->makeDefault();
						}
						line_index = getNextToken(line_index, token);
						if(!strcmp(token, ")")){
							line_index = getNextToken(line_index, token);
							break;
						}
						else if(!strcmp(token, ",")){
							line_index = getNextToken(line_index, token);
							if(!isAlphaNumeric(token[0])){
								fprintf(stderr, "Line %d: Expected attribute name after ','\n", line_count);
								error = 1;
								break;
							}
						}
						else{
							fprintf(stderr, "Line %d: Expected ',' or ')'\n", line_count);
							error = 1;
							break;
						}
					} while(line_index != NULL);
				}

				if(!error)
				{
					if(is_default){
						if(line_index != NULL){
							fprintf(stderr, "Line %d: Expected EOL after string\n", line_count); 
							error = 1;
						}
					}
					else{
						// expecting :
						if(!strcmp(token, ":")) {
							line_index = getNextToken(line_index, token);
							int token_len = strlen(token);
							if(!checkStringToken(token, token_len, ":", line_count)){
								// writing resource name to output header file
								if(num_resources != 0)
									fputs(",\n", output_header);
								fprintf(output_header, "\t%s=%d", resource_id, num_resources);
								// writing resource descriptor to output cpp file
								if(num_resources != 0)
									fputs(",\n", output_cpp);
								if(token_len > 2)
									token[token_len-1] = '\0';
								fprintf(output_cpp, "\tResourceDescriptor(%d, \"%s%s\", %d, %d, ResourceOptions(ResourceOptions::",
													resource_type, path_prefix, token+1, start_count, end_count);
								fprintf(output_cpp, "%s(", RESOURCE_TYPE_NAMES[resource_type]);
								// writing attributes
								std::vector<ResourceAttribute>::iterator it = attribs[resource_type].begin();
								while(it != attribs[resource_type].end())
								{
									if(it != attribs[resource_type].begin())
										fputs(",", output_cpp);
									it->print(output_cpp);
									// resetting attribute to default value
									it->reset();
									it++;
								}
								fputs(")))", output_cpp);
								num_resources++;
								// expecting EOL
								if(getNextToken(line_index, token)){
									fprintf(stderr, "Line %d: Expected EOL\n", line_count); 
									error = 1;
								}

							}else{
								error = 1;
							}
						}
						else{
							fprintf(stderr, "Line %d: Expected ':' after resource id\n", line_count);
							error = 1;
						}
					}
				}
			}
			else{
				fprintf(stderr, "Line %d: Expected resource id after '%s' specification\n", line_count, resource_name);
				error = 1;
			}
		}

		if(error){
			fprintf(stderr, "Code generation failed!\n");
			fclose(input);
			fclose(output_cpp);
			fclose(output_header);
			return 1;
		}

	}


	// write header postfix
	fputs(HEADER_POST, output_header);
	fprintf(output_header, "\n#define RESOURCES_NUMBER %d\n", num_resources);
	fputs("extern const ResourceDescriptor RESOURCES_DESCRIPTOR[RESOURCES_NUMBER];\n#endif", output_header);

	// write cpp postfix
	fputs(CPP_POST, output_cpp);

	// close every file
	fclose(input);
	fclose(output_cpp);
	fclose(output_header);

	return 0;
}
