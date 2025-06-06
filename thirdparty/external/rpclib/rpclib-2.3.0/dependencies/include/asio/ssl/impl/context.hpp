//
// ssl/impl/context.hpp
// ~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2005 Voipster / Indrek dot Juhani at voipster dot com
// Copyright (c) 2005-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_SSL_IMPL_CONTEXT_HPP
#define ASIO_SSL_IMPL_CONTEXT_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"

#if !defined(ASIO_ENABLE_OLD_SSL)
# include "asio/detail/throw_error.hpp"
#endif // !defined(ASIO_ENABLE_OLD_SSL)

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {
namespace ssl {

#if !defined(ASIO_ENABLE_OLD_SSL)

template <typename VerifyCallback>
void context::set_verify_callback(VerifyCallback callback)
{
  clmdep_asio::error_code ec;
  this->set_verify_callback(callback, ec);
  clmdep_asio::detail::throw_error(ec, "set_verify_callback");
}

template <typename VerifyCallback>
clmdep_asio::error_code context::set_verify_callback(
    VerifyCallback callback, clmdep_asio::error_code& ec)
{
  return do_set_verify_callback(
      new detail::verify_callback<VerifyCallback>(callback), ec);
}

template <typename PasswordCallback>
void context::set_password_callback(PasswordCallback callback)
{
  clmdep_asio::error_code ec;
  this->set_password_callback(callback, ec);
  clmdep_asio::detail::throw_error(ec, "set_password_callback");
}

template <typename PasswordCallback>
clmdep_asio::error_code context::set_password_callback(
    PasswordCallback callback, clmdep_asio::error_code& ec)
{
  return do_set_password_callback(
      new detail::password_callback<PasswordCallback>(callback), ec);
}

#endif // !defined(ASIO_ENABLE_OLD_SSL)

} // namespace ssl
} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // ASIO_SSL_IMPL_CONTEXT_HPP
