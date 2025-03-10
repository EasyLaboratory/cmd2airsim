//
// detail/reactor_op.hpp
// ~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_DETAIL_REACTOR_OP_HPP
#define ASIO_DETAIL_REACTOR_OP_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"
#include "asio/detail/operation.hpp"

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {
namespace detail {

class reactor_op
  : public operation
{
public:
  // The error code to be passed to the completion handler.
  clmdep_asio::error_code ec_;

  // The number of bytes transferred, to be passed to the completion handler.
  std::size_t bytes_transferred_;

  // Perform the operation. Returns true if it is finished.
  bool perform()
  {
    return perform_func_(this);
  }

protected:
  typedef bool (*perform_func_type)(reactor_op*);

  reactor_op(perform_func_type perform_func, func_type complete_func)
    : operation(complete_func),
      bytes_transferred_(0),
      perform_func_(perform_func)
  {
  }

private:
  perform_func_type perform_func_;
};

} // namespace detail
} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // ASIO_DETAIL_REACTOR_OP_HPP
