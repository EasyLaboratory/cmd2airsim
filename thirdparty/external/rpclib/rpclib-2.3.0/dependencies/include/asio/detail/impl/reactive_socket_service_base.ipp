//
// detail/reactive_socket_service_base.ipp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_DETAIL_IMPL_REACTIVE_SOCKET_SERVICE_BASE_IPP
#define ASIO_DETAIL_IMPL_REACTIVE_SOCKET_SERVICE_BASE_IPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"

#if !defined(ASIO_HAS_IOCP) \
  && !defined(ASIO_WINDOWS_RUNTIME)

#include "asio/detail/reactive_socket_service_base.hpp"

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {
namespace detail {

reactive_socket_service_base::reactive_socket_service_base(
    clmdep_asio::io_service& io_service)
  : reactor_(use_service<reactor>(io_service))
{
  reactor_.init_task();
}

void reactive_socket_service_base::shutdown_service()
{
}

void reactive_socket_service_base::construct(
    reactive_socket_service_base::base_implementation_type& impl)
{
  impl.socket_ = invalid_socket;
  impl.state_ = 0;
}

void reactive_socket_service_base::base_move_construct(
    reactive_socket_service_base::base_implementation_type& impl,
    reactive_socket_service_base::base_implementation_type& other_impl)
{
  impl.socket_ = other_impl.socket_;
  other_impl.socket_ = invalid_socket;

  impl.state_ = other_impl.state_;
  other_impl.state_ = 0;

  reactor_.move_descriptor(impl.socket_,
      impl.reactor_data_, other_impl.reactor_data_);
}

void reactive_socket_service_base::base_move_assign(
    reactive_socket_service_base::base_implementation_type& impl,
    reactive_socket_service_base& other_service,
    reactive_socket_service_base::base_implementation_type& other_impl)
{
  destroy(impl);

  impl.socket_ = other_impl.socket_;
  other_impl.socket_ = invalid_socket;

  impl.state_ = other_impl.state_;
  other_impl.state_ = 0;

  other_service.reactor_.move_descriptor(impl.socket_,
      impl.reactor_data_, other_impl.reactor_data_);
}

void reactive_socket_service_base::destroy(
    reactive_socket_service_base::base_implementation_type& impl)
{
  if (impl.socket_ != invalid_socket)
  {
    ASIO_HANDLER_OPERATION(("socket", &impl, "close"));

    reactor_.deregister_descriptor(impl.socket_, impl.reactor_data_,
        (impl.state_ & socket_ops::possible_dup) == 0);

    clmdep_asio::error_code ignored_ec;
    socket_ops::close(impl.socket_, impl.state_, true, ignored_ec);
  }
}

clmdep_asio::error_code reactive_socket_service_base::close(
    reactive_socket_service_base::base_implementation_type& impl,
    clmdep_asio::error_code& ec)
{
  if (is_open(impl))
  {
    ASIO_HANDLER_OPERATION(("socket", &impl, "close"));

    reactor_.deregister_descriptor(impl.socket_, impl.reactor_data_,
        (impl.state_ & socket_ops::possible_dup) == 0);
  }

  socket_ops::close(impl.socket_, impl.state_, false, ec);

  // The descriptor is closed by the OS even if close() returns an error.
  //
  // (Actually, POSIX says the state of the descriptor is unspecified. On
  // Linux the descriptor is apparently closed anyway; e.g. see
  //   http://lkml.org/lkml/2005/9/10/129
  // We'll just have to assume that other OSes follow the same behaviour. The
  // known exception is when Windows's closesocket() function fails with
  // WSAEWOULDBLOCK, but this case is handled inside socket_ops::close().
  construct(impl);

  return ec;
}

clmdep_asio::error_code reactive_socket_service_base::cancel(
    reactive_socket_service_base::base_implementation_type& impl,
    clmdep_asio::error_code& ec)
{
  if (!is_open(impl))
  {
    ec = clmdep_asio::error::bad_descriptor;
    return ec;
  }

  ASIO_HANDLER_OPERATION(("socket", &impl, "cancel"));

  reactor_.cancel_ops(impl.socket_, impl.reactor_data_);
  ec = clmdep_asio::error_code();
  return ec;
}

clmdep_asio::error_code reactive_socket_service_base::do_open(
    reactive_socket_service_base::base_implementation_type& impl,
    int af, int type, int protocol, clmdep_asio::error_code& ec)
{
  if (is_open(impl))
  {
    ec = clmdep_asio::error::already_open;
    return ec;
  }

  socket_holder sock(socket_ops::socket(af, type, protocol, ec));
  if (sock.get() == invalid_socket)
    return ec;

  if (int err = reactor_.register_descriptor(sock.get(), impl.reactor_data_))
  {
    ec = clmdep_asio::error_code(err,
        clmdep_asio::error::get_system_category());
    return ec;
  }

  impl.socket_ = sock.release();
  switch (type)
  {
  case SOCK_STREAM: impl.state_ = socket_ops::stream_oriented; break;
  case SOCK_DGRAM: impl.state_ = socket_ops::datagram_oriented; break;
  default: impl.state_ = 0; break;
  }
  ec = clmdep_asio::error_code();
  return ec;
}

clmdep_asio::error_code reactive_socket_service_base::do_assign(
    reactive_socket_service_base::base_implementation_type& impl, int type,
    const reactive_socket_service_base::native_handle_type& native_socket,
    clmdep_asio::error_code& ec)
{
  if (is_open(impl))
  {
    ec = clmdep_asio::error::already_open;
    return ec;
  }

  if (int err = reactor_.register_descriptor(
        native_socket, impl.reactor_data_))
  {
    ec = clmdep_asio::error_code(err,
        clmdep_asio::error::get_system_category());
    return ec;
  }

  impl.socket_ = native_socket;
  switch (type)
  {
  case SOCK_STREAM: impl.state_ = socket_ops::stream_oriented; break;
  case SOCK_DGRAM: impl.state_ = socket_ops::datagram_oriented; break;
  default: impl.state_ = 0; break;
  }
  impl.state_ |= socket_ops::possible_dup;
  ec = clmdep_asio::error_code();
  return ec;
}

void reactive_socket_service_base::start_op(
    reactive_socket_service_base::base_implementation_type& impl,
    int op_type, reactor_op* op, bool is_continuation,
    bool is_non_blocking, bool noop)
{
  if (!noop)
  {
    if ((impl.state_ & socket_ops::non_blocking)
        || socket_ops::set_internal_non_blocking(
          impl.socket_, impl.state_, true, op->ec_))
    {
      reactor_.start_op(op_type, impl.socket_,
          impl.reactor_data_, op, is_continuation, is_non_blocking);
      return;
    }
  }

  reactor_.post_immediate_completion(op, is_continuation);
}

void reactive_socket_service_base::start_accept_op(
    reactive_socket_service_base::base_implementation_type& impl,
    reactor_op* op, bool is_continuation, bool peer_is_open)
{
  if (!peer_is_open)
    start_op(impl, reactor::read_op, op, true, is_continuation, false);
  else
  {
    op->ec_ = clmdep_asio::error::already_open;
    reactor_.post_immediate_completion(op, is_continuation);
  }
}

void reactive_socket_service_base::start_connect_op(
    reactive_socket_service_base::base_implementation_type& impl,
    reactor_op* op, bool is_continuation,
    const socket_addr_type* addr, size_t addrlen)
{
  if ((impl.state_ & socket_ops::non_blocking)
      || socket_ops::set_internal_non_blocking(
        impl.socket_, impl.state_, true, op->ec_))
  {
    if (socket_ops::connect(impl.socket_, addr, addrlen, op->ec_) != 0)
    {
      if (op->ec_ == clmdep_asio::error::in_progress
          || op->ec_ == clmdep_asio::error::would_block)
      {
        op->ec_ = clmdep_asio::error_code();
        reactor_.start_op(reactor::connect_op, impl.socket_,
            impl.reactor_data_, op, is_continuation, false);
        return;
      }
    }
  }

  reactor_.post_immediate_completion(op, is_continuation);
}

} // namespace detail
} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // !defined(ASIO_HAS_IOCP)
       //   && !defined(ASIO_WINDOWS_RUNTIME)

#endif // ASIO_DETAIL_IMPL_REACTIVE_SOCKET_SERVICE_BASE_IPP
