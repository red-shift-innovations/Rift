#pragma once

#include <iosfwd>
#include <string>

#include <rift/export.hpp>

namespace rift
{
  // Print a greeting for the specified name into the specified
  // stream. Throw std::invalid_argument if the name is empty.
  //
  RIFT_SYMEXPORT void
  say_hello (std::ostream&, const std::string& name);
}
