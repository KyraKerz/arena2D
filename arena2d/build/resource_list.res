# comment

### how to use ###
# define image resource
#img IMAGE_NAME: "path/to/image"

# define sound resource
#snd SOUND_NAME: "path/to/sound"

# define font resource
#font FONT_NAME: "path/to/font"

# define resource collection (loads images "path/to/image000.png", "path/to/image001.png", ... , "path/to/image010.png")
#img IMAGE_COLLECTION_NAME[0-10]: "path/to/image$$$.png"

# image load settings
#img IMAGE_FOO(pre_mult_alpha=true, gen_mipmap=true, wrap=REPEAT): "path/to/image"
#################

# setting default 
img DEFAULT(pre_mult_alpha=true, gen_mipmap=true, wrap=CLAMP)

# global
prefix: "data/fonts/"
fnt FONT_MONOSPACE_REGULAR: "Bitstream_Regular.ttf"
fnt FONT_MONOSPACE_BOLD: "Bitstream_Bold.ttf"
