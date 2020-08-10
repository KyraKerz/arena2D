#include "zVector4d.h"

void zVector4D::print(const char * s) const
{
	if(s == NULL)
		print("v");
	else
		printf("%s: x=%f, y=%f, z=%f, w=%f\n",s, x, y, z, w);
}