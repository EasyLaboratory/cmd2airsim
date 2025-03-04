//
// signal_set_service.hpp
// ~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_SIGNAL_SET_SERVICE_HPP
#define ASIO_SIGNAL_SET_SERVICE_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"
#include "asio/async_result.hpp"
#include "asio/detail/signal_set_service.hpp"
#include "asio/error.hpp"
#include "asio/io_service.hpp"

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {

/// Default service implementation for a signal set.
class signal_set_service
#if defined(GENERATING_DOCUMENTATION)
  : public clmdep_asio::io_service::service
#else
  : public clmdep_asio::detail::service_base<signal_set_service>
#endif
{
public:
#if defined(GENERATING_DOCUMENTATION)
  /// The unique service identifier.
  static clmdep_asio::io_service::id id;
#endif

public:
  /// The type of a signal set implementation.
#if defined(GENERATING_DOCUMENTATION)
  typedef implementation_defined implementation_type;
#else
  typedef detail::signal_set_service::implementation_type implementation_type;
#endif

  /// Construct a new signal set service for the specified io_service.
  explicit signal_set_service(clmdep_asio::io_service& io_service)
    : clmdep_asio::detail::service_base<signal_set_service>(io_service),
      service_impl_(io_service)
  {
  }

  /// Construct a new signal set implementation.
  void construct(implementation_type& impl)
  {
    service_impl_.construct(impl);
  }

  /// Destroy a signal set implementation.
  void destroy(implementation_type& impl)
  {
    service_impl_.destroy(impl);
  }

  /// Add a signal to a signal_set.
  clmdep_asio::error_code add(implementation_type& impl,
      int signal_number, clmdep_asio::error_code& ec)
  {
    return service_impl_.add(impl, signal_number, ec);
  }

  /// Remove a signal to a signal_set.
  clmdep_asio::error_code remove(implementation_type& impl,
      int signal_number, clmdep_asio::error_code& ec)
  {
    return service_impl_.remove(impl, signal_number, ec);
  }

  /// Remove all signals from a signal_set.
  clmdep_asio::error_code clear(implementation_type& impl,
      clmdep_asio::error_code& ec)
  {
    return service_impl_.clear(impl, ec);
  }

  /// Cancel all operations associated with the signal set.
  clmdep_asio::error_code cancel(implementation_type& impl,
      clmdep_asio::error_code& ec)
  {
    return service_impl_.cancel(impl, ec);
  }

  // Start an asynchronous operation to wait for a signal to be delivered.
  template <typename SignalHandler>
  ASIO_INITFN_RESULT_TYPE(SignalHandler,
      void (clmdep_asio::error_code, int))
  async_wait(implementation_type& impl,
      ASIO_MOVE_ARG(SignalHandler) handler)
  {
    detail::async_result_init<
      SignalHandler, void (clmdep_asio::error_code, int)> init(
        ASIO_MOVE_CAST(SignalHandler)(handler));

    service_impl_.async_wait(impl, init.handler);

    return init.result.get();
  }

private:
  // Destroy all user-defined handler objects owned by the service.
  void shutdown_service()
  {
    service_impl_.shutdown_service();
  }

  // Perform any fork-related housekeeping.
  void fork_service(clmdep_asio::io_service::fork_event event)
  {
    service_impl_.fork_service(event);
  }

  // The platform-specific implementation.
  detail::signal_set_service service_impl_;
};

} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // ASIO_SIGNAL_SET_SERVICE_HPP
