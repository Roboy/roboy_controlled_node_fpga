//
// Limit.h
//
// $Id: //poco/1.3/Data/include/Poco/Data/Limit.h#1 $
//
// Library: Data
// Package: DataCore
// Module:  Limit
//
// Definition of the Limit class.
//
// Copyright (c) 2006, Applied Informatics Software Engineering GmbH.
// and Contributors.
//
// Permission is hereby granted, free of charge, to any person or organization
// obtaining a copy of the software and accompanying documentation covered by
// this license (the "Software") to use, reproduce, display, distribute,
// execute, and transmit the Software, and to prepare derivative works of the
// Software, and to permit third-parties to whom the Software is furnished to
// do so, all subject to the following:
// 
// The copyright notices in the Software and this entire statement, including
// the above license grant, this restriction and the following disclaimer,
// must be included in all copies of the Software, in whole or in part, and
// all derivative works of the Software, unless such copies or derivative
// works are solely in the form of machine-executable object code generated by
// a source language processor.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
// SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
// FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//


#ifndef Data_Limit_INCLUDED
#define Data_Limit_INCLUDED


#include "Poco/Data/Data.h"


namespace Poco {
namespace Data {


class Data_API Limit
	/// Limit stores information how many rows a query should return.
{
public:
	enum
	{
		LIMIT_UNLIMITED = 0xffffffffu
	};
	
	Limit(Poco::UInt32 value, bool hardLimit, bool isLowerLimit);
		/// Creates the Limit. 
		///
		/// Value contains the upper row hint, if hardLimit is set to true, the limit acts as a hard
		/// border, ie. every query must return exactly value rows, returning more than value objects will throw an exception!
		/// LowerLimits always act as hard-limits!
		///
		/// A value of LIMIT_UNLIMITED disables the limit.

	~Limit();
		/// Destroys the Limit.

	Poco::UInt32 value() const;
		/// Returns the value of the limit

	bool isHardLimit() const;
		/// Returns true if the limit is a hard limit.

	bool isLowerLimit() const;
		/// Returns true if the limit is a lower limit, otherwise it is an upperLimit

private:
	Poco::UInt32 _value;
	bool         _hardLimit;
	bool         _isLowerLimit;
};


//
// inlines
//
inline Poco::UInt32 Limit::value() const
{
	return _value;
}


inline bool Limit::isHardLimit() const
{
	return _hardLimit;
}


inline bool Limit::isLowerLimit() const
{
	return _isLowerLimit;
}


} } // namespace Poco::Data


#endif // Data_Limit_INCLUDED
