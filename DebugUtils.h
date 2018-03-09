#pragma once

#define DEBUG_APP

	#ifdef DEBUG_APP
		#define DEBUG_APP_PRINT(...) Serial.print(__VA_ARGS__)
		#define DEBUG_APP_PRINTLN(...) Serial.println(__VA_ARGS__)
	#else
		#define DEBUG_APP_PRINT(...)
		#define DEBUG_APP_PRINTLN(...)
	#endif
