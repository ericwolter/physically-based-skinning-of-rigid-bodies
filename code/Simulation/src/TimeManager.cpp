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

#include "TimeManager.h"

using namespace IBDS;

TimeManager* TimeManager::current = 0;

/** Konstruktor: Initialisiert die Variablen.
  */
TimeManager::TimeManager () 
{
	time = 0;
	h = 0.01;
}

/** Destruktor
  */
TimeManager::~TimeManager () 
{
	current = 0;
}

/** Die Klasse TimeManager ist ein Singleton. Die Funktion
  * getCurrent gibt die einzige Instanz des Singletons zurück. 
  * Falls nötig wird eine neue Instanz angelegt.
  */
TimeManager* TimeManager::getCurrent ()
{
	if (current == 0)
	{
		current = new TimeManager ();
	}
	return current;
}


/** Setzt die aktuelle Instanz des Singletons TimeManager.
  */
void TimeManager::setCurrent (TimeManager* tm)
{
	current = tm;
}

/** hasCurrent gibt zurück, ob die einzige Instanz der Klasse angelegt wurde.
  */
bool TimeManager::hasCurrent()
{
	return (current != 0);
}

/** Gibt die aktuelle Zeit zurück.
  */
Real TimeManager::getTime ()
{
	return time;
}

/** Setzt die aktuelle Zeit.
  */
void TimeManager::setTime (Real t)
{
	time = t;
}

/** Gibt die aktuell gesetzte Weite eines Zeitschrittes zurück.
  */
Real TimeManager::getTimeStepSize ()
{
	return h;
}

/** Setzt die Weite eines Zeitschrittes.
  */
void TimeManager::setTimeStepSize (Real tss)
{
	h = tss;
}
