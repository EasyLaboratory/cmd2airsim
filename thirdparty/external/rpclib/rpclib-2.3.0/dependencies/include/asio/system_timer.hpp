//
// system_timer.hpp
// ~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_SYSTEM_TIMER_HPP
#define ASIO_SYSTEM_TIMER_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"

#if defined(ASIO_HAS_STD_CHRONO) \
  || defined(ASIO_HAS_BOOST_CHRONO) \
  || defined(GENERATING_DOCUMENTATION)

#if defined(ASIO_HAS_STD_CHRONO)
# include <chrono>
#elif defined(ASIO_HAS_BOOST_CHRONO)
# include <boost/chrono/system_clocks.hpp>
#endif

#include "asio/basic_waitable_timer.hpp"

namespace clmdep_asio {

#if defined(GENERATING_DOCUMENTATION)
/// Typedef for a timer based on the system clock.
/**
 * This typedef uses the C++11 @c &lt;chrono&gt; standard library facility, if
 * available. Otherwise, it may use the Boost.Chrono library. To explicitly
 * utilise Boost.Chrono, use the basic_waitable_timer template directly:
 * @code
 * typedef basic_waitable_timer<boost::chrono::system_clock> timer;
 * @endcode
 */
typedef basic_waitable_timer<chrono::system_clock> system_timer;
#elif defined(ASIO_HAS_STD_CHRONO)
typedef basic_waitable_timer<std::chrono::system_clock> system_timer;
#elif defined(ASIO_HAS_BOOST_CHRONO)
typedef basic_waitable_timer<boost::chrono::system_clock> system_timer;
#endif

} // namespace clmdep_asio

#endif // defined(ASIO_HAS_STD_CHRONO) 
       //   || defined(ASIO_HAS_BOOST_CHRONO)
       //   || defined(GENERATING_DOCUMENTATION)

#endif // ASIO_SYSTEM_TIMER_HPP
