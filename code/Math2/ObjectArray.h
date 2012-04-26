/*
 * IBDS - Impulse-Based Dynamic Simulation Library
 * Copyright (c) 2003-2010 Jan Bender http://www.impulse-based.de
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


#ifndef ObjectArray_H
#define ObjectArray_H

#include "Common/Config.h"
#include <memory.h>

template <class T, int growBy = 100, bool linearGrow = true>
class ObjectArray
{
private:
	unsigned int m_size;
	unsigned int m_capacity;
	T* m_data;

protected:
	FORCE_INLINE void init()
	{
		m_size = 0u;
		m_capacity = 0u;
		m_data = NULL;
	}

public:
	FORCE_INLINE ObjectArray()
	{
		init();
	}

	~ObjectArray()
	{
		clear();
	}

	/** Return the pointer of the data.
	 */
	FORCE_INLINE T* arrayPointer()
	{
		return m_data;
	}

	/** Return the pointer of the data.
	 */
	FORCE_INLINE const T* arrayPointer() const 
	{
		return m_data;
	}

	/** Set size to zero but do not change capacity.
	 */
	FORCE_INLINE void reset()
	{
		m_size = 0u;
	}

	FORCE_INLINE void clear()
	{
		delete [] m_data;
		init();
	}

	/** Return the number of elements
	 */
	FORCE_INLINE unsigned int size() const
	{	
		return m_size;
	}

	/** Return the capacity
	 */
	FORCE_INLINE unsigned int capacity() const
	{	
		return m_capacity;
	}

	FORCE_INLINE T* getData()
	{
		return m_data;
	}

	FORCE_INLINE const T* getData() const 
	{
		return m_data;
	}

	T& create()
	{
		if(m_size >= m_capacity)
			grow();
		return m_data[m_size++];
	}

	/** Insert element at the given index. 
	 */
	FORCE_INLINE T& insert(const unsigned int index, const T& data)
	{
		if(m_size >= m_capacity)
			grow();
		for(unsigned int i=m_size; i > index; i++)
		{
			m_data[i] = m_data[i-1];
		}
		m_size++;
		m_data[index] = data;
	}

	/** Remove the element at the given index. 
	 */
	FORCE_INLINE void removeAt(const unsigned int index)
	{
		m_data[index].~T();
		for(unsigned int i=index+1u; i < m_size; i++)
		{
			m_data[i-1] = m_data[i];
		}
		m_size--;
	}

	FORCE_INLINE void pop_back()
	{
		m_size--;
		m_data[m_size].~T();
	}

	FORCE_INLINE void push_back(const T& data)
	{
		if(m_size >= m_capacity)
			grow();
		m_data[m_size] = data;
		m_size++;
	}

	FORCE_INLINE T& operator[](const unsigned int i)
	{
		return m_data[i];
	}

	FORCE_INLINE const T& operator[](const unsigned int i) const
	{
		return m_data[i];
	}

	FORCE_INLINE void reserve(const unsigned int count)
	{
		if (!m_data)
		{
			unsigned int newSize = count;
			if(newSize < growBy)
				newSize = growBy;
			m_data = new T[newSize];
			m_size = 0;
			m_capacity = newSize;
		}
		else
		{
			if(count <= m_capacity)
				return;
			unsigned int newSize = m_capacity;
			newSize = (linearGrow) ? (newSize+growBy) : (newSize*2u);
			if (count < newSize)
			{
				grow();
				return;
			}
			T* tmp = new T[count];
			memcpy(tmp, m_data, sizeof(T)*m_size);
			delete [] m_data;
			m_data = tmp;
			m_capacity = count;
		}
	}

	FORCE_INLINE void resize(unsigned int count)
	{
		reserve(count);
		m_size = count;
	}

private:
	FORCE_INLINE void grow()
	{
		if (!m_data)
		{
			m_data = new T[growBy];
			m_size = 0;
			m_capacity = growBy;
		}
		else if (linearGrow)
		{
			const unsigned int newSize = m_capacity + growBy;
			T* tmp = new T[newSize];
			memcpy(tmp, m_data, sizeof(T)*m_size);
			delete [] m_data;
			m_data = tmp;
			m_capacity = newSize;
		}
		else
		{
			const unsigned int newSize = m_capacity*2u;
			T* tmp = new T[newSize];
			memcpy(tmp, m_data, sizeof(T)*m_size);
			delete [] m_data;
			m_data = tmp;
			m_capacity = newSize;
		}
	}
};

#endif
