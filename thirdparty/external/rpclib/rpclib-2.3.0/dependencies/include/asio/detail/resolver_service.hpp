//
// detail/resolver_service.hpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_DETAIL_RESOLVER_SERVICE_HPP
#define ASIO_DETAIL_RESOLVER_SERVICE_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"

#if !defined(ASIO_WINDOWS_RUNTIME)

#include "asio/ip/basic_resolver_iterator.hpp"
#include "asio/ip/basic_resolver_query.hpp"
#include "asio/detail/addressof.hpp"
#include "asio/detail/resolve_endpoint_op.hpp"
#include "asio/detail/resolve_op.hpp"
#include "asio/detail/resolver_service_base.hpp"

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {
namespace detail {

template <typename Protocol>
class resolver_service : public resolver_service_base
{
public:
  // The implementation type of the resolver. A cancellation token is used to
  // indicate to the background thread that the operation has been cancelled.
  typedef socket_ops::shared_cancel_token_type implementation_type;

  // The endpoint type.
  typedef typename Protocol::endpoint endpoint_type;

  // The query type.
  typedef clmdep_asio::ip::basic_resolver_query<Protocol> query_type;

  // The iterator type.
  typedef clmdep_asio::ip::basic_resolver_iterator<Protocol> iterator_type;

  // Constructor.
  resolver_service(clmdep_asio::io_service& io_service)
    : resolver_service_base(io_service)
  {
  }

  // Resolve a query to a list of entries.
  iterator_type resolve(implementation_type&, const query_type& query,
      clmdep_asio::error_code& ec)
  {
    clmdep_asio::detail::addrinfo_type* address_info = 0;

    socket_ops::getaddrinfo(query.host_name().c_str(),
        query.service_name().c_str(), query.hints(), &address_info, ec);
    auto_addrinfo auto_address_info(address_info);

    return ec ? iterator_type() : iterator_type::create(
        address_info, query.host_name(), query.service_name());
  }

  // Asynchronously resolve a query to a list of entries.
  template <typename Handler>
  void async_resolve(implementation_type& impl,
      const query_type& query, Handler& handler)
  {
    // Allocate and construct an operation to wrap the handler.
    typedef resolve_op<Protocol, Handler> op;
    typename op::ptr p = { clmdep_asio::detail::addressof(handler),
      clmdep_asio_handler_alloc_helpers::allocate(
        sizeof(op), handler), 0 };
    p.p = new (p.v) op(impl, query, io_service_impl_, handler);

    ASIO_HANDLER_CREATION((p.p, "resolver", &impl, "async_resolve"));

    start_resolve_op(p.p);
    p.v = p.p = 0;
  }

  // Resolve an endpoint to a list of entries.
  iterator_type resolve(implementation_type&,
      const endpoint_type& endpoint, clmdep_asio::error_code& ec)
  {
    char host_name[NI_MAXHOST];
    char service_name[NI_MAXSERV];
    socket_ops::sync_getnameinfo(endpoint.data(), endpoint.size(),
        host_name, NI_MAXHOST, service_name, NI_MAXSERV,
        endpoint.protocol().type(), ec);

    return ec ? iterator_type() : iterator_type::create(
        endpoint, host_name, service_name);
  }

  // Asynchronously resolve an endpoint to a list of entries.
  template <typename Handler>
  void async_resolve(implementation_type& impl,
      const endpoint_type& endpoint, Handler& handler)
  {
    // Allocate and construct an operation to wrap the handler.
    typedef resolve_endpoint_op<Protocol, Handler> op;
    typename op::ptr p = { clmdep_asio::detail::addressof(handler),
      clmdep_asio_handler_alloc_helpers::allocate(
        sizeof(op), handler), 0 };
    p.p = new (p.v) op(impl, endpoint, io_service_impl_, handler);

    ASIO_HANDLER_CREATION((p.p, "resolver", &impl, "async_resolve"));

    start_resolve_op(p.p);
    p.v = p.p = 0;
  }
};

} // namespace detail
} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // !defined(ASIO_WINDOWS_RUNTIME)

#endif // ASIO_DETAIL_RESOLVER_SERVICE_HPP
