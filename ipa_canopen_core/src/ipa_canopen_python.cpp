#include <boost/python.hpp>

// char const* yay()
// {
//   return "Yay!";
// }

#include "ipa_canopen_core/canopen.h"

BOOST_PYTHON_MODULE(ipa_canopen)
{
  using namespace boost::python;
//   def("init", yay);
}
