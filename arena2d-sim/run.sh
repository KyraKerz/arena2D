#!/bin/bash
bash ./build.sh
if [ $? -eq "0" ]; then
	echo "--- RUNNING APPLICATION ---"
	./build/arena
fi
