//
// detail/reactive_null_buffers_op.hpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_DETAIL_REACTIVE_NULL_BUFFERS_OP_HPP
#define ASIO_DETAIL_REACTIVE_NULL_BUFFERS_OP_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"
#include "asio/detail/addressof.hpp"
#include "asio/detail/fenced_block.hpp"
#include "asio/detail/handler_alloc_helpers.hpp"
#include "asio/detail/handler_invoke_helpers.hpp"
#include "asio/detail/reactor_op.hpp"

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {
namespace detail {

template <typename Handler>
class reactive_null_buffers_op : public reactor_op
{
public:
  ASIO_DEFINE_HANDLER_PTR(reactive_null_buffers_op);

  reactive_null_buffers_op(Handler& handler)
    : reactor_op(&reactive_null_buffers_op::do_perform,
        &reactive_null_buffers_op::do_complete),
      handler_(ASIO_MOVE_CAST(Handler)(handler))
  {
  }

  static bool do_perform(reactor_op*)
  {
    return true;
  }

  static void do_complete(io_service_impl* owner, operation* base,
      const clmdep_asio::error_code& /*ec*/,
      std::size_t /*bytes_transferred*/)
  {
    // Take ownership of the handler object.
    reactive_null_buffers_op* o(static_cast<reactive_null_buffers_op*>(base));
    ptr p = { clmdep_asio::detail::addressof(o->handler_), o, o };

    ASIO_HANDLER_COMPLETION((o));

    // Make a copy of the handler so that the memory can be deallocated before
    // the upcall is made. Even if we're not about to make an upcall, a
    // sub-object of the handler may be the true owner of the memory associated
    // with the handler. Consequently, a local copy of the handler is required
    // to ensure that any owning sub-object remains valid until after we have
    // deallocated the memory here.
    detail::binder2<Handler, clmdep_asio::error_code, std::size_t>
      handler(o->handler_, o->ec_, o->bytes_transferred_);
    p.h = clmdep_asio::detail::addressof(handler.handler_);
    p.reset();

    // Make the upcall if required.
    if (owner)
    {
      fenced_block b(fenced_block::half);
      ASIO_HANDLER_INVOCATION_BEGIN((handler.arg1_, handler.arg2_));
      clmdep_asio_handler_invoke_helpers::invoke(handler, handler.handler_);
      ASIO_HANDLER_INVOCATION_END;
    }
  }

private:
  Handler handler_;
};

} // namespace detail
} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // ASIO_DETAIL_REACTIVE_NULL_BUFFERS_OP_HPP
