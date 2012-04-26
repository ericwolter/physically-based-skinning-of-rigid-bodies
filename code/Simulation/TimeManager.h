/*
 * IBDS - Impulse-Based Dynamic Simulation Library
 * Copyright (c) 2003-2008 Jan Bender http://www.impulse-based.de
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Jan Bender - Jan.Bender@impulse-based.de
 */

#ifndef _TIMEMANAGER_H
#define _TIMEMANAGER_H

#include "Common/Config.h"

namespace IBDS
{
	/** Diese Klasse verwaltet die Simulationszeit. 
	  \author Jan Bender
	  */
	class TimeManager
	{
	private:
		/** Aktuelle Zeit */
		Real time;
		/** Einzige Instanz des TimeManager */
		static TimeManager *current;
		/** Aktuelle Größe eines Zeitschrittes */
		Real h;

	public:
		TimeManager ();
		~TimeManager ();

		// Singleton
		static TimeManager* getCurrent ();
		static void setCurrent (TimeManager* tm);
		static bool hasCurrent();

		Real getTime ();
		void setTime (Real t);
		Real getTimeStepSize ();
		void setTimeStepSize (Real tss);
	};
}

#endif
