#ifndef APP_MODE_H
#define APP_MODE_H
//application modes
//use CONSOLE for plain console applications (e.g. server)
//use VISUAL_THREADED prevents SDL event processing from blocking so game logic (running in separate thread) can continue without rendering
enum class ApplicationMode{CONSOLE, VISUAL};

#endif
