#ifndef TOOLS_HPP
#define TOOLS_HPP

// NAOqi Headers
#include <qi/anyvalue.hpp>

std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value, std::vector<float>& result);

std::vector<int> fromAnyValueToIntVector(qi::AnyValue& value, std::vector<int>& result);

#endif // TOOLS_HPP
