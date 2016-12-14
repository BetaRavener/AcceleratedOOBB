/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file Log.cpp
	@author Jukka Jylänki
	@brief The LOG and LOGUSER macros. Provides an unified mechanism for logging. */
#include "MathLog.h"

#include <cstring>
#include <cstdarg>
#include <stdio.h>

void PrintToConsole(MathLogChannel channel, const char *str)
{
	printf("%s\n", str);
}

void PrintToConsoleVariadic(MathLogChannel channel, const char *format, ...)
{
	const int capacity = 2048;
	char str[capacity];

	va_list args;
	va_start(args, format);

	vsnprintf((char *)str, capacity, format, args);
	str[capacity-1] = 0; // We only support logging a fixed-length string so don't care if we fail/truncate, just make sure we zero-terminate so there won't be any issues.
	PrintToConsole(channel, str);

	va_end(args);
}
